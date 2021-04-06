#include <algorithm>
#include <cmath>
#include <cassert>
#include <sys/stat.h>
#include <fstream>
#include <fcntl.h>
#include <iostream>
#include <wait.h>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/trim.hpp>

#include "Util.h"

struct configInfo gConfig;

std::vector<struct blockEventInfo> readBlockFile(std::string &blockFile) {
  std::vector<struct blockEventInfo> blockEvents;

  if(blockFile.empty())
    return blockEvents;

  std::ifstream reader;
  reader.open(blockFile);
  std::string line;

  while(std::getline(reader, line)) {
    auto sp = split(line, ",");
    size_t startTime;
    std::string blockLaneID;
    std::string rerouteLaneID;

    if(sp.size() > 2) {
      blockLaneID = sp[0];
      boost::algorithm::trim(blockLaneID);
      rerouteLaneID = sp[2];
      boost::algorithm::trim(rerouteLaneID);
      try {
        std::string num = sp[1];
        boost::algorithm::trim(num);
        startTime = std::stoul(num);
      }
      catch(std::exception &e) {
        std::cerr << "Can not read block file " << blockFile <<
                  " line " << line << ", stoi failed!" << std::endl;
        exit(-1);
      }
    } else {
      std::cerr << "Can not read block file " << blockFile <<
                " line \"" << line << "\", format error!" << std::endl;
      exit(-1);
    }

    std::cout << "Found block event of lane " << blockLaneID << " at time step " << startTime << std::endl;

    struct blockEventInfo blockEvent;
    blockEvent.blockLaneID = blockLaneID;
    blockEvent.timeStep = startTime;
    blockEvent.rerouteLaneID = rerouteLaneID;

    blockEvents.push_back(blockEvent);
  }

  return blockEvents;
}

std::set<std::string> readIgnoreFiles(std::vector<std::string> &ignoreFiles) {
  std::set<std::string> ignoreIDs;

  for(const auto &filename : ignoreFiles) {
    std::ifstream reader;
    reader.open(filename);
    std::string line;

    while(std::getline(reader, line)) {
      // support commend with #
      if(line[0]=='#')
        continue;

      auto sp = split(line, "#");
      if(sp.size() > 1) {
        line = sp[0];
      }

      boost::algorithm::trim(line);
      ignoreIDs.insert(line);
    }
  }

  return ignoreIDs;
}

void parse_args(int argc, char *argv[], struct configInfo &config) {

  std::string blockFile;
  std::vector<std::string> ignoreFiles;
  std::vector<std::string> shieldIDFile;

  try {
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("sumo,c", boost::program_options::value(&config.sumoConfigFile)->required(), "SUMO config file.")
        ("port,p", boost::program_options::value(&config.port), "Port number for TraCI.")
        ("shield,s", boost::program_options::value(&config.shieldConfigFiles)->multitoken(), "Shield config files.")
        ("out,o", boost::program_options::value(&config.logFile), "Outfile name for shielding log.")
        ("whitelist-tls,w", boost::program_options::value(&shieldIDFile),
            "File with junction or tls IDs which will be shielded.")
        ("blacklist-tls,b", boost::program_options::value(&ignoreFiles)->multitoken(),
            "File with junction or tls IDs which will be not shielded.")
        ("incident-events,i", boost::program_options::value(&blockFile),
            "File with traffic indecent event config.")
        ("update-interval,u", boost::program_options::value(&config.updateInterval), "Update interval of shields.")
        ("lambda,l", boost::program_options::value(&config.lambda), "Learning rate (Shield Parameter lambda).")
        ("param-d,d", boost::program_options::value(&config.d), "Shield Parameter d.")
        ("max-lane-size,k", boost::program_options::value(&config.maxLaneSize),
            "Limit of recognized waiting vehicles on lanes (Shield Parameter K).")
        ("simulation-time,t", boost::program_options::value(&config.simulationTime),
            "Time step until stop the simulation.")
        ("warm-up-time,x", boost::program_options::value(&config.warmUpTime),
            "Time step until the shield starts to intervene.")
        ("gui,g", "Use sumo-gui.")
        ("free,f", "Run without Shields.")
        ("bus", "Prioritize public transport.")
        ("no-lane-merging", "Avoid the merging of parallel lanes.")
        ("static-update", "Do shield updates in static interval no Minimum change for update required.")
        ("no-lane-trees", "Disable accurate lane state for non OSM Maps.")
        ("side-by-side", "Run a shielded and unshielded simulation simulations.")
        ("hook-sumo", "Connect to external started SUMO.")
        ("overwrite-controller",
         "Overwrite the traffic light controller (RL Agent) with the shield strategy "
         "and reset to previous action if the overwritten controller takes not the control back.")
        ("help", "Help message.");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

    if(vm.count("help") || argc==1) {
      std::cout << desc << "\n";
      exit(1);
    }

    boost::program_options::notify(vm);

    config.unshielded = vm.count("free") ? true : false;
    config.sideBySide = vm.count("side-by-side") ? true : false;
    config.noMerging = vm.count("no-lane-merging") ? true : false;
    config.staticUpdate = vm.count("static-update") ? true : false;
    config.gui = vm.count("gui") ? true : false;
    config.prioritizeBus = vm.count("bus") ? true : false;
    config.noTrees = vm.count("no-lane-trees") ? true : false;
    config.client = vm.count("hook-sumo") ? true : false;
    config.overwrite = vm.count("overwrite-controller") ? true : false;
  }
  catch(std::exception &e) {
    std::cout << e.what() << "\n";
    exit(1);
  }

  if(fileExist(config.sumoConfigFile) && !config.client) {
    std::cerr << "Sumo Config File " << config.sumoConfigFile << " does not exist\n";
    exit(1);
  }

  if(!blockFile.empty() && fileExist(blockFile)) {
    std::cerr << "block File " << blockFile << " does not exist\n";
    exit(1);
  }

  for(const auto &file : config.shieldConfigFiles) {
    if(fileExist(file)) {
      std::cerr << "Shield Config File " << file << " does not exist\n";
      exit(1);
    }
  }

  for(const auto &file : ignoreFiles) {
    if(fileExist(file)) {
      std::cerr << "Ignore File " << file << " does not exist\n";
      exit(1);
    }
  }

  for(const auto &file : shieldIDFile) {
    if(fileExist(file)) {
      std::cerr << "Shield File " << file << " does not exist\n";
      exit(1);
    }
  }

  if(!blockFile.empty()) {
    config.blockEvents = readBlockFile(blockFile);
  }

  if(!ignoreFiles.empty()) {
    config.ignoreIDs = readIgnoreFiles(ignoreFiles);
  }

  if(!shieldIDFile.empty()) {
    config.shieldedIDs = readIgnoreFiles(shieldIDFile);
  }
}

void pythonPlot(const std::vector<char *> &logFiles) {
  pid_t pid, w;
  pid = fork();
  int waitStatus;
  if(pid==-1) {
    std::cerr << "Could not fork to start python\n";
  }

  if(pid==0) {
    std::vector<char *> args;
    args.push_back(const_cast<char *>("/usr/bin/python3"));
    args.push_back(const_cast<char *>("plot.py"));
    for(auto &logFile : logFiles)
      args.push_back(logFile);
    args.push_back(NULL);

    int fd = open("python.log", O_RDWR | O_CREAT | O_APPEND,
                  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);

    dup2(fd, STDOUT_FILENO);
    dup2(fd, STDERR_FILENO);

    execv(args[0], args.data());
    perror("PYTHON NOT STARTED!\n");
  } else {
    do {
      w = waitpid(pid, &waitStatus, WUNTRACED | WCONTINUED);
      if(w==-1) {
        std::cerr << "Could not wait for process\n";
      }
      if(WIFEXITED(waitStatus) && waitStatus==0) {
        std::cout << "python succeed\n";
      } else if(waitStatus!=0) {
        std::cerr << "python did not succeed\n";
        exit(0);
      } else if(WIFSIGNALED(waitStatus)) {
        printf("killed by signal %d\n", WTERMSIG(waitStatus));
      } else if(WIFSTOPPED(waitStatus)) {
        printf("stopped by signal %d\n", WSTOPSIG(waitStatus));
      } else if(WIFCONTINUED(waitStatus)) {
        printf("continued\n");
      }
    } while(!WIFEXITED(waitStatus) && !WIFSIGNALED(waitStatus));
  }
}

bool isNear(float value, float reference) {
  return std::abs(value - reference) < EPSILON;
}

bool isPMF(std::vector<float> probabilities) {
  float sum = 0;
  std::for_each(probabilities.begin(), probabilities.end(), [&](float p) { sum += p; });
  return isNear(sum, 1);
}

std::string getSubstrBetweenDelims(const std::string &str,
                                   const std::string &start_delim,
                                   const std::string &stop_delim) {

  unsigned first_delim_pos = str.find(start_delim);
  if(first_delim_pos > str.length()) {
    return std::string("");
  }

  unsigned end_pos_of_first_delim = first_delim_pos + start_delim.length();
  unsigned last_delim_pos = first_delim_pos + str.substr(first_delim_pos, str.length() - 1).find(stop_delim);

  return str.substr(end_pos_of_first_delim, last_delim_pos - end_pos_of_first_delim);
}

std::string getValueForToken(std::string const &str, std::string const &token) {
  return getSubstrBetweenDelims(str, token + "=", "\t&");
}

std::vector<float> calculatePMF(std::vector<int> &tracking) {
  float length = 0;
  std::for_each(tracking.begin(), tracking.end(), [&](auto n) { length += n; });
  std::vector<float> probabilities;
  for(auto track : tracking) {
    probabilities.push_back(track/float(length));
  }

  // assert(isPMF(probabilities));
  return probabilities;
}
bool fileExist(const std::string &path) {
  struct stat buffer{};
  if(stat(path.c_str(), &buffer)!=0) {
    return true;
  }
  return false;
}

std::vector<std::string> split(const std::string &line, const std::string &delim) {
  std::size_t current, previous = 0;
  std::vector<std::string> ret;

  current = line.find(delim);
  while(current!=std::string::npos) {
    ret.push_back(line.substr(previous, current - previous));
    previous = current + 1;
    current = line.find(delim, previous);
  }
  ret.push_back(line.substr(previous, current - previous));

  return ret;
}

void replace(std::string &str, const std::string &from, const std::string &to) {
  size_t start_pos = str.find(from);
  if(start_pos!=std::string::npos) {
    str.replace(start_pos, from.length(), to);
  }
}
void formatName(std::string &name) {
  if(name[0]=='-')
    name[0] = '_';

  if(name[1]=='-')
    name[1] = '_';

  replace(name, "#", "RRR");
  name = "lane" + name;
}
void reformatName(std::string &name) {
  name = name.substr(4);
  replace(name, "RRR", "#");

  if(name[1]=='_')
    name[1] = '-';

  if(name[0]=='_')
    name[0] = '-';
}

std::string getTimeString() {
  time_t t;
  char buffer[80];
  time(&t);
  strftime(buffer, 80, "%Y-%m-%dT%H:%M:%S", localtime(&t));
  return std::string(buffer);
}
