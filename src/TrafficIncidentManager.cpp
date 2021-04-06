#include <cassert>

#include "TrafficIncidentManager.h"

DynamicReroute::DynamicReroute(SUMOConnector &sumo) : sumo(sumo) {}

void DynamicReroute::addRerouting(const std::string &laneID) {
  laneIDs.push_back(laneID);
}

void DynamicReroute::step() {
  for(const auto &laneID : laneIDs) {
    auto vehIDs = sumo.getLaneLastStepVehicleIDs(laneID);
    for(const auto &vehID : vehIDs) {
      if(std::find(reroutedIDs.begin(), reroutedIDs.end(), vehID)==reroutedIDs.end()) {
        try {
          sumo.rerouteVehicle(vehID);
          reroutedIDs.push_back(vehID);
        }
        catch(std::exception &e) {
          continue;
        }
      }
    }
  }
}

TrafficIncidentManager::TrafficIncidentManager(SUMOConnector &sumo,
                                               const std::vector<struct blockEventInfo> &blockEvents) :
    DynamicReroute(sumo), blockEvents(blockEvents) {}

void TrafficIncidentManager::step() {
  auto sumoTimeStep = sumo.getTime();

  for(const auto &blockEvent : blockEvents) {
    if(blockEvent.timeStep==sumoTimeStep) {
      sumo.blockLane(blockEvent.blockLaneID);
      addRerouting(blockEvent.rerouteLaneID);
    }
  }

  DynamicReroute::step();
}
