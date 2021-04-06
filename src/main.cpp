#include <vector>
#include <string>
#include <unistd.h>

#include "SUMOConnector.h"
#include "Simulation.h"

void syncGui(SUMOConnector &sumo1, SUMOConnector &sumo2);

int main(int argc, char *argv[]) {
  parse_args(argc, argv, gConfig);

  std::vector<Simulation *> simulations;

  if(gConfig.client) {
    simulations.push_back(new Simulation(gConfig.sumoConfigFile,
                                         gConfig.logFile,
                                         gConfig.shieldConfigFiles,
                                         gConfig.ignoreIDs,
                                         gConfig.shieldedIDs,
                                         gConfig.blockEvents,
                                         true,
                                         !gConfig.unshielded,
                                         false));
  } else
  if(!gConfig.sideBySide) {
    if(gConfig.logFile.empty()) {
      gConfig.logFile = !gConfig.unshielded ? "log.shield" : "log.free";
    }
    simulations.push_back(new Simulation(gConfig.sumoConfigFile,
                                         gConfig.logFile,
                                         gConfig.shieldConfigFiles,
                                         gConfig.ignoreIDs,
                                         gConfig.shieldedIDs,
                                         gConfig.blockEvents,
                                         false,
                                         !gConfig.unshielded,
                                         gConfig.gui));
  } else {
    simulations.push_back(new Simulation(gConfig.sumoConfigFile,
                                         (gConfig.logFile.empty() ? "sbs_shielded.log" : gConfig.logFile),
                                         gConfig.shieldConfigFiles,
                                         gConfig.ignoreIDs,
                                         gConfig.shieldedIDs,
                                         gConfig.blockEvents,
                                         false,
                                         true,
                                         gConfig.gui,
                                         1330,
                                         960, 1080,
                                         0, 0));

    simulations.push_back(new Simulation(gConfig.sumoConfigFile,
                                         (gConfig.logFile.empty() ? "sbs_free.log" : gConfig.logFile + ".free"),
                                         gConfig.shieldConfigFiles,
                                         gConfig.ignoreIDs,
                                         gConfig.shieldedIDs,
                                         gConfig.blockEvents,
                                         false,
                                         false,
                                         gConfig.gui,
                                         1331,
                                         960, 1080,
                                         960, 0));
  }

  for(int step = 0; step < gConfig.simulationTime; step++) {
    for(auto &simulation : simulations) {
      simulation->step();
    }

    if(gConfig.gui && simulations.size()==2) {
      syncGui(simulations[0]->getSumoInstance(), simulations[1]->getSumoInstance());
    }
  }

  if(gConfig.gui) {
    std::vector<char *> logFiles;
    for(auto &simulation : simulations) {
      logFiles.push_back(simulation->getLogFile());
    }
    pythonPlot(logFiles);
  }

  sleep(1);

  for(auto &simulation : simulations) {
    std::cout << "Simulation " << simulation->getLogFile() << std::endl;
    std::cout << "Performance : " << simulation->getSumoInstance().getPerformance() << std::endl;
    std::cout << "Total deviation count : " << simulation->getSumoInstance().getTotalDeviation()
              << " - " << simulation->getSumoInstance().getDeviationPercentage() << "%" << std::endl;

    delete simulation;
  }

  return 0;
}

void syncGui(SUMOConnector &sumo1, SUMOConnector &sumo2) {
  // NOTE mirror only gui1 to gui2.

  bool changedGuiZoom = false;
  bool changedGuiOffset = false;

  auto guiZoom = sumo1.getGuiZoom();
  auto guiOffset = sumo1.getGuiOffset();

  if(sumo1.currentGuiZoom!=guiZoom)
    changedGuiZoom = true;

  if(sumo1.currentGuiOffset.x!=guiOffset.x || sumo1.currentGuiOffset.y!=guiOffset.y)
    changedGuiOffset = true;

  sumo1.currentGuiZoom = guiZoom;
  sumo1.currentGuiOffset = guiOffset;

  if(changedGuiZoom) {
    // std::cout << "Adapt zoom " << guiZoom << std::endl;
    sumo2.setGuiZoom(guiZoom);
  }

  if(changedGuiOffset) {
    // std::cout << "Adapt offset " << guiOffset.x << ", " << guiOffset.y << std::endl;
    sumo2.setGuiOffset(guiOffset);
  }
}
