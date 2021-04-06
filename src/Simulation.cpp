#include "Simulation.h"
#include "TrafficLight.h"

Simulation::Simulation(const std::string &sumoConfigFile,
                       const std::string &simulationLogFile,
                       const std::vector<std::string> &shieldConfigFiles,
                       const std::set<std::string> &ignoreIDs,
                       const std::set<std::string> &shieldedIDs,
                       const std::vector<struct blockEventInfo> &blockEvents,
                       bool client,
                       bool shield,
                       bool gui,
                       int port,
                       int xSize,
                       int ySize,
                       int xPos,
                       int yPos)
    : sumo(sumoConfigFile, simulationLogFile, port, gui),
      tim(sumo, blockEvents),
      simulationLogFile(simulationLogFile),
      ignoreIDs(ignoreIDs),
      shieldedIDs(shieldedIDs),
      client(client) {

  sumo.setSumoGuiWindowSize(xSize, ySize);
  sumo.setSumoGuiWindowPos(xPos, yPos);
  if(!client)
    sumo.boot();

  sumo.connect();

  clock_t simulationInitTime = clock();

  if(shield) {
    auto tlsIDs = sumo.getTrafficLightIDs();

    if(shieldConfigFiles.empty()) {
      for(const auto &tlsID : tlsIDs) {

        if(!shieldedIDs.empty()) {
          if(!containsTlsID(shieldedIDs, tlsID))
            continue;
          std::cout << "SHIELD tlsID " << tlsID << "!" << std::endl;
        } else {
          // ignore tls and junctions
          if(containsTlsID(ignoreIDs, tlsID)) {
            std::cout << "IGNORE tlsID " << tlsID << " and will not be shielded!" << std::endl;
            continue;
          }
        }

        try {
          auto *t = TrafficLight::build(&sumo, tlsID);
          trafficLight.push_back(t);
        }
        catch(std::exception &e) {
          std::cout << "Traffic Light " << tlsID << "contains unsupported lane scenario, it will be disabled1"
                    << std::endl;
        }

      }
    } else {
      for(const std::string &filename : shieldConfigFiles) {
        std::cout << "Parsing " << filename << "\n";

        try {
          auto *t = TrafficLight::buildFromFile(&sumo, filename);
          trafficLight.push_back(t);
        }
        catch(std::exception &e) {
          std::cout << "Traffic Light config " << filename
                    << "contains unsupported lane scenario, it will be disabled!" << std::endl;
        }
      }
    }

    std::cout << "Simulation Init Time: " << float(clock() - simulationInitTime)/CLOCKS_PER_SEC << std::endl;
  }
}

Simulation::~Simulation() {
  for(auto tl : trafficLight) {
    delete tl;
  }

  sumo.closeAndExit();
}

SUMOConnector &Simulation::getSumoInstance() {
  return sumo;
}

char *Simulation::getLogFile() {
  return const_cast<char *>(simulationLogFile.c_str());
}

void Simulation::step() {

  sumo.step();
  tim.step();

  for(auto &tl : trafficLight) {
    tl->step();
  }
}

void Simulation::loop() {

  clock_t cycleTime;
  while(sumo.getTimeStep() < gConfig.simulationTime) {
    cycleTime = clock();
    step();
    std::cout << "STEP TIME: " << float(clock() - cycleTime)/CLOCKS_PER_SEC << std::endl;
  }

  std::cout << "Performance : " << sumo.getPerformance() << std::endl;
  std::cout << "Total deviation count : "
            << sumo.getTotalDeviation() << " - "
            << 100.f*(float)sumo.getTotalDeviation()/(float)sumo.getTimeStep() << "%"
            << std::endl;
}

bool Simulation::containsTlsID(const std::set<std::string> &listOfIDs, const std::string &tlsID) {
  if(!listOfIDs.empty()) {
    // try tlsID
    if(listOfIDs.find(tlsID)!=listOfIDs.end()) {
      //std::cout << "IGNORE tlsID " << tlsID << " and will not be shielded!" << std::endl;
      return true;
    }

    // try matching junctionID
    for(const auto &junctionID : sumo.getTlsJunctions(tlsID))
      if(listOfIDs.find(junctionID)!=listOfIDs.end()) {
        //std::cout << "IGNORE tlsID " << tlsID << " and will not be shielded!" << std::endl;
        return true;
      }
  }

  return false;
}

