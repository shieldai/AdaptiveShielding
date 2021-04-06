#include <wait.h>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>

#include "STORMConnector.h"
#include "Shield.h"

void STORMConnector::startStrategyUpdate(Shield *shield) {
  // check if there is a active job
  if(jobs.find(shield)!=jobs.end()) {
    return;
  }

  auto start = clock();
  auto tlsID = shield->getJunction();
  pid_t pid = startStorm(tlsID);

  jobs[shield].pid = pid;
  jobs[shield].start = start;

  while(checkOnStorm(shield)==-1);
}

pid_t STORMConnector::startStorm(std::string &filePrefix) {
  // std::cout << "START STORM for junction " << junction << std::endl;
  std::vector<std::string> vecArgs;
  vecArgs.push_back("--prism");
  vecArgs.push_back(out_path_ + filePrefix + ".prism");
  vecArgs.push_back("--prop");
  vecArgs.push_back(out_path_ + filePrefix + ".props");
  vecArgs.push_back("--exportscheduler");
  vecArgs.push_back(out_path_ + filePrefix + ".sched");
  vecArgs.push_back("--buildstateval");
  vecArgs.push_back("--buildchoicelab");
  vecArgs.push_back("--timeout");
  vecArgs.push_back("180");
  const char *args[vecArgs.size() + 2];

  args[0] = "/usr/bin/storm";
  for(uint i = 1; i < vecArgs.size() + 1; i++) {
    args[i] = vecArgs.at(i - 1).c_str();
  }
  args[vecArgs.size() + 1] = nullptr;

  pid_t pid = fork();
  if(pid==0) {
    try {
      int fd = open("STORM.log", O_RDWR | O_CREAT | O_APPEND,
                    S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);
      dup2(fd, STDOUT_FILENO);
      dup2(fd, STDERR_FILENO);
    } catch(std::exception &e) {
      std::cerr << e.what() << std::endl;
    }

    execv(args[0], (char **)args);
    perror("STORM NOT STARTED!\n");

  } else if(pid==-1) {
    std::cerr << "Could not fork to start storm process, going to use old strategy (if there is one..)\n";
    return pid;
  }

  return pid;
}

int STORMConnector::checkOnStorm(Shield *shield) {

  if(jobs.find(shield)==jobs.end()) {
    return 1;
  }

  pid_t pid = jobs[shield].pid;
  clock_t start = jobs[shield].start;

  int waitStatus;
  pid_t w = waitpid(pid, &waitStatus, WUNTRACED | WCONTINUED);
  if(w==-1) {
    std::cerr << "Could not wait for process\n";
    return 1;
  }
  if(WIFEXITED(waitStatus) && waitStatus==0) {
    // std::cout << "Storm PID " << pid << " for " << junction << " success after "
    //   << float(clock() - start)/CLOCKS_PER_SEC << std::endl;

    shield->updateStrategyCallback();
    jobs.erase(shield);
    return 0;
  } else if(waitStatus!=0) {
    std::cerr << "Storm PID " << pid << " for " << shield->getJunction() <<
              " did not success after " << float(clock() - start)/CLOCKS_PER_SEC << std::endl;
    // avoid state space growth
    shield->lockStateSpaceSize();
    jobs.erase(shield);
    return 1;
  } else if(WIFSIGNALED(waitStatus)) {
    printf("Storm PID %d killed by signal %d\n", pid, WTERMSIG(waitStatus));
    jobs.erase(shield);
    return 1;
  } else if(WIFSTOPPED(waitStatus)) {
    printf("Storm PID %d stopped by signal %d\n", pid, WSTOPSIG(waitStatus));
    jobs.erase(shield);
    return 1;
  } else if(WIFCONTINUED(waitStatus)) {
    //printf("continued\n");
  }

  return -1;
}
