#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <cassert>

#include "SUMOConnector.h"

SUMOConnector::SUMOConnector(const std::string &config,
                             const std::string &logFile = "",
                             int port = 1330,
                             bool gui = false)
    : port_(port), useGui(gui), config_(config), ip_address_("localhost") {
  if(logFile.empty()) {
    filename_ = std::experimental::filesystem::path(config).filename();
    filename_ += ".log";
  } else {
    filename_ = logFile;
  }

  log = std::ofstream(filename_, std::ofstream::out | std::ofstream::trunc);
}

void SUMOConnector::boot() {
  pid_t pid = fork();
  if(pid==-1) {
    std::cerr << "Could not fork to start SUMO, aborting\n";
    return;
  }
  if(pid==0) {

    std::string argPort = std::to_string(port_);
    std::string argWindowSize = "--window-size=";
    std::string argWindowPos = "--window-pos=";

    std::vector<char *> args;
    if(useGui) {
      args.push_back(const_cast<char *>("/usr/bin/sumo-gui"));
    } else {
      args.push_back(const_cast<char *>("/usr/bin/sumo"));
    }

    args.push_back(const_cast<char *>("-c"));
    args.push_back(const_cast<char *>(config_.c_str()));
    args.push_back(const_cast<char *>("--remote-port"));
    args.push_back(const_cast<char *>(argPort.c_str()));
    args.push_back(const_cast<char *>("--start"));
    args.push_back(const_cast<char *>("-Q"));
    args.push_back(const_cast<char *>("--no-step-log=true"));
    args.push_back(const_cast<char *>("--time-to-teleport=-1"));

    if(useGui && sumoGuiWindowSize.first!=-1 && sumoGuiWindowSize.second!=-1) {
      argWindowSize += std::to_string(sumoGuiWindowSize.first) + "," + std::to_string(sumoGuiWindowSize.second);
      args.push_back(const_cast<char *>(argWindowSize.c_str()));
    }

    if(useGui && sumoGuiWindowPos.first!=-1 && sumoGuiWindowPos.second!=-1) {
      argWindowPos += std::to_string(sumoGuiWindowPos.first) + "," + std::to_string(sumoGuiWindowPos.second);
      args.push_back(const_cast<char *>(argWindowPos.c_str()));
    }

    args.push_back(NULL);

    int fd = open("stderr.log", O_RDWR | O_CREAT | O_APPEND,
                  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);

    dup2(fd, STDOUT_FILENO);
    dup2(fd, STDERR_FILENO);

    execv(args[0], args.data());
    perror("SUMO NOT STARTED!\n");
  } else {
    pid_ = pid;
  }
}

void SUMOConnector::connect() {
  do {
    try {
      TraCIAPI::connect(ip_address_, port_);
      setOrder(42);
      break;
    }
    catch(const tcpip::SocketException &e) {
      sleep(1);
      std::cerr << "Trying to connect\n";
    }
  } while(true);

  setupSubscribe();
  mapJunctionToTls();

  auto edgeCount = edge.getIDCount();
  auto laneCount = lane.getIDCount();
  auto trafficLightCount = trafficlights.getIDCount();

  size_t maxLane = 0;
  std::string maxLaneName;
  size_t maxEdge = 0;
  std::string maxEdgeName;

  for(const auto &tl : trafficLightIDs) {
    auto controlledLanes = trafficlights.getControlledLanes(tl);
    if(controlledLanes.size() > maxLane) {
      maxLane = controlledLanes.size();
      maxLaneName = tl;
    }

    std::set<std::string> edgeIDs;
    for(const auto &l : controlledLanes) {
      edgeIDs.insert(lane.getEdgeID(l));
    }
    if(edgeIDs.size() > maxEdge) {
      maxEdge = edgeIDs.size();
      maxEdgeName = tl;
    }
  }

  std::string out;
  out += "Map " + config_ + "\n";
  out += "Edges " + std::to_string(edgeCount) + ", Lanes " + std::to_string(laneCount) + "\n";
  out += "Traffic Lights " + std::to_string(trafficLightCount) + "\n";
  out += "\tTraffic Lights with max Lanes " + maxLaneName
      + " = " + std::to_string(maxLane) + "\n";
  out += "\tTraffic Lights with max Edges " + maxEdgeName
      + " = " + std::to_string(maxEdge) + "\n";

  std::cout << out;
}

void SUMOConnector::closeAndExit() {
  std::cout << "Closing SUMO Connection\n";
  std::cout << costs << std::endl;
  close();

  if(pid_ != -1)
    kill(pid_, SIGKILL);
}

void SUMOConnector::track() {
  departedVehicles = simulation.getDepartedIDList();
  arrivedVehicles = simulation.getArrivedIDList();

  // REMOVE OUTGOING
  for(const auto &rm : arrivedVehicles) {
    auto it = vehicles.find(rm);
    if(it!=vehicles.end())
      vehicles.erase(it);

    it = filteredVehicles.find(rm);
    if(it!=filteredVehicles.end())
      filteredVehicles.erase(it);
  }

  vehResult = vehicle.getAllSubscriptionResults();
  //edgeResult = edge.getAllSubscriptionResults();
  laneResult = lane.getAllSubscriptionResults();
  tlResult = trafficlights.getAllSubscriptionResults();
  if(useGui) {
    guiResult = gui.getAllSubscriptionResults();
  }

  totalWaitingTime = 0;
  totalAccWaitingTime = 0;
  filteredVehiclesTotalWaitingTime = 0;
  filteredVehiclesTotalAccWaitingTime = 0;
  filteredVehiclesHaltingNumberPerLane.clear();
  filteredVehiclesVehicleNumberPerLane.clear();
  if(!vehicles.empty()) {
    for(const auto &vehID : vehicles) {

      if(!filteredVehicles.empty() && filteredVehicles.find(vehID)!=filteredVehicles.end()) {
        auto laneID = getVehicleLaneID(vehID);

        auto waitingTime = getVehicleWaitingTime(vehID);
        auto accWaitingTime = getVehicleAccWaitingTime(vehID);

        std::string vLaneID = virtualLaneID(laneID);
        if(gConfig.prioritizeBus) {
          // IN THIS CASE JUST MAP IT TO THE NORMAL LANE ID
          vLaneID = laneID;
        }

        auto expLaneID = vehIDtoLaneID[vehID];
        if(expLaneID!=laneID) {
          filteredAccVehiclesHaltingNumberPerLane[expLaneID] = 0;
          vehIDtoLaneID[vehID] = laneID;
        } else {
          if(waitingTime > 0.0) {
            filteredAccVehiclesHaltingNumberPerLane[laneID]++;
          }
        }

        filteredVehiclesVehicleNumberPerLane[vLaneID]++;
        if(waitingTime > 0.0) {
          filteredVehiclesHaltingNumberPerLane[vLaneID]++;
        }

        filteredVehiclesTotalWaitingTime += waitingTime;
        filteredVehiclesTotalAccWaitingTime += accWaitingTime;
        continue;
      }

      totalWaitingTime += getVehicleWaitingTime(vehID);
      totalAccWaitingTime += getVehicleAccWaitingTime(vehID);
    }
    totalWaitingTimePerVehicle = totalWaitingTime/(double)vehicles.size();
    totalAccWaitingTimePerVehicle = totalAccWaitingTime/(double)vehicles.size();
  }

  for(const auto &laneID : lanes) {
    totalHaltingNumber += getLaneLastStepHaltingNumber(laneID);
    totalVehicleNumber += getLaneLastStepVehicleNumber(laneID);
    totalMeanSpeed += getLaneLastMeanSpeed(laneID);
  }

  /// POLL ALL EDGES OR LANES!
  /* for(const auto& edgeID : edges) {
    totalHaltingNumber += getEdgeLastStepHaltingNumber(edgeID);
    totalVehicleNumber += getLaneLastStepVehicleNumber(edgeID);
    totalMeanSpeed += getLaneLastMeanSpeed(edgeID);
  } */

  costs += (double)totalHaltingNumber/(double)edges.size();

  for(const auto &vehID : departedVehicles) {
    vehicle.subscribe(vehID, vehicleVars, startSubscription, endSubscription);

    if(strncmp(vehicleFilterLabel.c_str(), vehID.c_str(), vehicleFilterLabel.size())==0) {
      filteredVehicles.insert(vehID);
    }
  }

  // ADD INGOING
  vehicles.insert(departedVehicles.begin(), departedVehicles.end());
}

void SUMOConnector::logSimulation() {
  log << simulation.getTime() << ",";
  log << vehicles.size() << ",";
  log << departedVehicles.size() << ",";
  log << arrivedVehicles.size() << ",";
  log << getHaltingNumber() << ",";
  log << getVehicleNumber() << ",";
  log << getMeanSpeed() << ",";
  log << getPerformance() << ",";
  log << totalWaitingTimePerVehicle << ",";
  log << totalAccWaitingTimePerVehicle << ",";
  log << totalWaitingTime << ",";
  log << totalAccWaitingTime << ",\t";
  log << filteredVehicles.size() << ",";
  log << filteredVehiclesTotalWaitingTime << ",";
  log << filteredVehiclesTotalAccWaitingTime << "\n";
  log.flush();
}

void SUMOConnector::mapJunctionToTls() {
  for(const auto &tlsID : trafficLightIDs) {
    // Try if tlsID matches junctionID
    if(std::find(junctions.begin(), junctions.end(), tlsID)!=junctions.end()) {
      tlsJunctions[tlsID].insert(tlsID);
      continue;
    }

    // Try if substring of tlsID matches junctionID
    for(const auto &junctionID : junctions) {
      if(tlsID.find(junctionID)!=std::string::npos) {
        tlsJunctions[tlsID].insert(junctionID);
      }
    }

    if(!tlsJunctions[tlsID].empty()) {
      continue;
    }

    // Try if iterate over the controlled links and take the viaLanes
    auto tlsLinks = trafficlights.getControlledLinks(tlsID);
    for(const auto &stateLink : tlsLinks) {
      for(const auto &link : stateLink) {
        for(const auto &junctionID : junctions) {
          if(link.viaLane.find(junctionID)!=std::string::npos) {
            tlsJunctions[tlsID].insert(junctionID);
          }
        }
      }
    }
  }
}

std::vector<std::string> SUMOConnector::findIncomingLanes(const std::string &nextLane) {
  std::set<std::string> incomingLanes;
  auto allLanes = getLaneIDs();

  for(auto &lane : allLanes) {
    auto link = laneLinks.at(lane);
    for(auto &l : link) {
      if(nextLane==l.approachedLane) {
        // FOUND PREV LANE!
        incomingLanes.insert(lane);
      }
    }
  }

  return std::vector<std::string>(incomingLanes.begin(), incomingLanes.end());
}

void SUMOConnector::setupSubscribe() {

  // static stuff
  lanes = lane.getIDList();
  edges = edge.getIDList();
  junctions = junction.getIDList();
  trafficLightIDs = trafficlights.getIDList();

  /* for(const auto& edgeID : edges) {
    edge.subscribe(edgeID, edgeVars, startSubscription, end);
  } */

  for(const auto &laneID : lanes) {
    lane.subscribe(laneID, laneVars, startSubscription, endSubscription);
  }

  for(const auto &tlsID : trafficLightIDs) {
    trafficlights.subscribe(tlsID, trafficLightVars, startSubscription, endSubscription);
  }

  for(auto &laneID : lanes) {
    laneLinks.insert(std::pair<std::string, std::vector<libsumo::TraCIConnection>>(laneID, lane.getLinks(laneID)));
  }

  if(useGui) {
    gui.subscribe(DEFAULT_VIEW, guiVars, startSubscription, endSubscription);
  }

  assert(lanes.size()==laneLinks.size());
}

void SUMOConnector::checkSubscriptionResults() {
  for(const auto &tlsID : trafficLightIDs) {
    assert(getTrafficLightCurrentPhase(tlsID)==trafficlights.getPhase(tlsID));
  }

  /* for(const auto& edgeID : edges) {
    assert(getEdgeLastStepVehicleNumber(edgeID) == edge.getLastStepVehicleNumber(edgeID));
    assert(getEdgeLastStepHaltingNumber(edgeID) == edge.getLastStepHaltingNumber(edgeID));
    assert(getEdgeLastMeanSpeed(edgeID) == edge.getLastStepMeanSpeed(edgeID));
  } */

  for(const auto &laneID : lanes) {
    assert(getLaneLastStepVehicleNumber(laneID)==lane.getLastStepVehicleNumber(laneID));
    assert(getLaneLastStepHaltingNumber(laneID)==lane.getLastStepHaltingNumber(laneID));
    assert(getLaneLastMeanSpeed(laneID)==lane.getLastStepMeanSpeed(laneID));
  }

  for(const auto &vehID : vehicles) {
    assert(getVehicleWaitingTime(vehID)==vehicle.getWaitingTime(vehID));
    assert(getVehicleAccWaitingTime(vehID)==vehicle.getAccumulatedWaitingTime(vehID));
  }
}

int SUMOConnector::getIntFromTraCIResult(libsumo::TraCIResult *result) {
  auto *rawPtr = dynamic_cast<libsumo::TraCIInt *>(result);
  // DGB assert(std::stoi(result->getString()) == rawPtr->value);
  return rawPtr->value;
}

double SUMOConnector::getDoubleFromTraCIResult(libsumo::TraCIResult *result) {
  auto *rawPtr = dynamic_cast<libsumo::TraCIDouble *>(result);
  // DBG assert(std::stod(result->getString()) == rawPtr->value);
  return rawPtr->value;
}

std::vector<std::string> SUMOConnector::getStringListFromTraCIResult(libsumo::TraCIResult *result) {
  auto *rawPtr = dynamic_cast<libsumo::TraCIStringList *>(result);
  // if(rawPtr==nullptr) return std::vector<std::string>();
  return rawPtr->value;
}

void SUMOConnector::blockLane(const std::string &laneID) {
  lane.setAllowed(laneID, {});
}

void SUMOConnector::step() {
  simulationStep();
  track();
  logSimulation();
  timeStep++;
  time++;
}

double SUMOConnector::getTimeStep() const {
  return timeStep;
}

double SUMOConnector::getTime() const {
  //assert(time==simulation.getTime());
  return time;
}

int SUMOConnector::getCurrentTime() const {
  //assert((int)time*1000==simulation.getCurrentTime());
  return (int)time*1000;
}

std::vector<std::string> SUMOConnector::getLaneIDs() {
  return lanes;
}

std::vector<std::string> SUMOConnector::getEdgeIDs() {
  return edges;
}

std::vector<std::string> SUMOConnector::getTrafficLightIDs() {
  return trafficLightIDs;
}

std::set<std::string> SUMOConnector::getVehicleIDs() {
  return vehicles;
}

int SUMOConnector::getTrafficLightCurrentPhase(const std::string &tlsID) {
  auto shPtr = tlResult[tlsID][libsumo::TL_CURRENT_PHASE];
  return getIntFromTraCIResult(shPtr.get());
}

std::string SUMOConnector::getTrafficLightCurrentProgram(const std::string &tlsID) {
  return tlResult[tlsID][libsumo::TL_CURRENT_PROGRAM]->getString();
}

std::vector<std::string> SUMOConnector::getTrafficLightsControlledLanes(const std::string &tlsID) const {
  auto cl = trafficlights.getControlledLanes(tlsID);

  // ADD VIRT LANE
  for(const auto &laneID : cl) {
    if(hasVirtualLane(laneID)) {
      // std::cout << "ADD VIRT LANE FOR " << tlsID << "(" << laneID << ")" << std::endl;
      cl.push_back(virtualLaneID(laneID));
    }
  }

  return cl;
}

std::vector<std::vector<libsumo::TraCILink>> SUMOConnector::getTrafficLightsControlledLinks(const std::string &tlsID) const {
  auto links = trafficlights.getControlledLinks(tlsID);
  return links;
}

std::vector<libsumo::TraCILogic> SUMOConnector::getTrafficLightsAllProgramLogics(const std::string &tlsID) const {
  return trafficlights.getAllProgramLogics(tlsID);
}

void SUMOConnector::setTrafficLightPhase(const std::string &tlsID, int phaseID) {
  trafficlights.setPhase(tlsID, phaseID);
}

void SUMOConnector::setTrafficLightProgram(const std::string &tlsID, const std::string &programID) {
  trafficlights.setProgram(tlsID, programID);
}

void SUMOConnector::setTrafficLightPhaseDuration(const std::string &tlsID, double phaseDuration) {
  trafficlights.setPhaseDuration(tlsID, phaseDuration);
}

double SUMOConnector::getLaneLength(const std::string &laneID) const {
  return lane.getLength(laneID);
}

int SUMOConnector::getLaneLastStepHaltingNumber(const std::string &laneID) {
  if(gConfig.prioritizeBus) {
    // IN THIS SETUP TAKE ONLY THE FILTERED VEHICLES
    return filteredAccVehiclesHaltingNumberPerLane[laneID];
  }

  if(isVirtualLaneID(laneID)) {
    //std::cout << "fetch HaltingNumber virt lane " << laneID << std::endl;
    return filteredVehiclesHaltingNumberPerLane[laneID];
  }

  auto shPtr = laneResult[laneID][libsumo::LAST_STEP_VEHICLE_HALTING_NUMBER];
  return getIntFromTraCIResult(shPtr.get());
}

int SUMOConnector::getLaneLastStepVehicleNumber(const std::string &laneID) {
  if(gConfig.prioritizeBus) {
    // IN THIS SETUP TAKE ONLY THE FILTERED VEHICLES
    return filteredVehiclesVehicleNumberPerLane[laneID];
  }

  if(isVirtualLaneID(laneID)) {
    //std::cout << "fetch VehicleNumber virt lane " << laneID << std::endl;
    return filteredVehiclesVehicleNumberPerLane[laneID];
  }

  auto shPtr = laneResult[laneID][libsumo::LAST_STEP_VEHICLE_NUMBER];
  return getIntFromTraCIResult(shPtr.get());
}

double SUMOConnector::getLaneLastMeanSpeed(const std::string &laneID) {
  auto shPtr = laneResult[laneID][libsumo::LAST_STEP_MEAN_SPEED];
  return getDoubleFromTraCIResult(shPtr.get());
}

std::vector<std::string> SUMOConnector::getLaneLastStepVehicleIDs(const std::string &laneID) {
  auto shPtr = laneResult[laneID][libsumo::LAST_STEP_VEHICLE_ID_LIST];
  return getStringListFromTraCIResult(shPtr.get());
}

int SUMOConnector::getEdgeLastStepHaltingNumber(const std::string &edgeID) {
  auto shPtr = edgeResult[edgeID][libsumo::LAST_STEP_VEHICLE_HALTING_NUMBER];
  return getIntFromTraCIResult(shPtr.get());
}

int SUMOConnector::getEdgeLastStepVehicleNumber(const std::string &edgeID) {
  auto shPtr = edgeResult[edgeID][libsumo::LAST_STEP_VEHICLE_NUMBER];
  return getIntFromTraCIResult(shPtr.get());
}

double SUMOConnector::getEdgeLastMeanSpeed(const std::string &edgeID) {
  auto shPtr = edgeResult[edgeID][libsumo::LAST_STEP_MEAN_SPEED];
  return getDoubleFromTraCIResult(shPtr.get());
}

double SUMOConnector::getVehicleWaitingTime(const std::string &vehID) {
  auto shPtr = vehResult[vehID][libsumo::VAR_WAITING_TIME];
  return getDoubleFromTraCIResult(shPtr.get());
}

double SUMOConnector::getVehicleAccWaitingTime(const std::string &vehID) {
  auto shPtr = vehResult[vehID][libsumo::VAR_ACCUMULATED_WAITING_TIME];
  return getDoubleFromTraCIResult(shPtr.get());
}

std::string SUMOConnector::getVehicleLaneID(const std::string &vehID) {
  return vehResult[vehID][libsumo::VAR_LANE_ID]->getString();
}

std::set<std::string> SUMOConnector::getTlsJunctions(const std::string &tlsID) {
  return tlsJunctions.at(tlsID);
}

libsumo::TraCIPosition SUMOConnector::getTlsPosition(const std::string &tlsID) {

  auto junctionIDs = tlsJunctions[tlsID];

  if(junctionIDs.size()==1)
    return junction.getPosition(*junctionIDs.begin());

  double totalX = 0, totalY = 0;
  for(const auto &junctionID : junctionIDs) {
    auto pos = junction.getPosition(junctionID);
    totalX += pos.x;
    totalY += pos.y;
  }

  libsumo::TraCIPosition pos;
  pos.x = totalX/junctionIDs.size();
  pos.y = totalY/junctionIDs.size();

  return pos;
}

double SUMOConnector::getPerformance() const {
  return costs;
}

size_t SUMOConnector::getHaltingNumber() const {
  return totalHaltingNumber;
}

size_t SUMOConnector::getVehicleNumber() const {
  return totalVehicleNumber;
}

double SUMOConnector::getMeanSpeed() const {
  return totalMeanSpeed;
}

void SUMOConnector::rerouteVehicle(const std::string &vehicleID) const {
  vehicle.rerouteTraveltime(vehicleID);
}

void SUMOConnector::createPoi(const std::string &poiID,
                              double x,
                              double y,
                              const libsumo::TraCIColor &c,
                              const std::string &type,
                              int layer,
                              const std::string &imgFile,
                              double width,
                              double height,
                              double angle) const {
  return poi.add(poiID, x, y, c, type, layer, imgFile, width, height, angle);
}

void SUMOConnector::setPoiColor(const std::string &poiID, const libsumo::TraCIColor &c) const {
  poi.setColor(poiID, c);
}

double SUMOConnector::getGuiZoom() {
  auto shPtr = guiResult[DEFAULT_VIEW][libsumo::VAR_VIEW_ZOOM];
  return getDoubleFromTraCIResult(shPtr.get());
}

libsumo::TraCIPosition SUMOConnector::getGuiOffset() {
  auto shPtr = guiResult[DEFAULT_VIEW][libsumo::VAR_VIEW_OFFSET];
  auto *rawPtr = dynamic_cast<libsumo::TraCIPosition *>(shPtr.get());
  return *rawPtr;
}

void SUMOConnector::setGuiZoom(double zoom) {
  gui.setZoom(DEFAULT_VIEW, zoom);
}
void SUMOConnector::setGuiOffset(const libsumo::TraCIPosition &offset) {
  gui.setOffset(DEFAULT_VIEW, offset.x, offset.y);
}

void SUMOConnector::setSumoGuiWindowSize(int x, int y) {
  sumoGuiWindowSize.first = x;
  sumoGuiWindowSize.second = y;
}

void SUMOConnector::setSumoGuiWindowPos(int x, int y) {
  sumoGuiWindowPos.first = x;
  sumoGuiWindowPos.second = y;
}

void SUMOConnector::addVirtualLane(const std::string &laneID) {
  virtualLanes.insert(laneID);
}

std::string SUMOConnector::virtualLaneID(const std::string &laneID) const {
  return vehicleFilterLabel + laneID;
}

bool SUMOConnector::isVirtualLaneID(const std::string &laneID) {
  if(strncmp(vehicleFilterLabel.c_str(), laneID.c_str(), vehicleFilterLabel.size())==0) {
    return true;
  }
  return false;
}

bool SUMOConnector::hasVirtualLane(const std::string &laneID) const {
  for(const auto &edgeID : virtualLanes) {
    int idx = laneID.find(edgeID);
    if(idx!=std::string::npos && idx==0) {
      return true;
    }
  }
  return false;
}

size_t SUMOConnector::getTotalDeviation() const {
  return totalDeviation;
}

float SUMOConnector::getDeviationPercentage() const {
  return 100.f*(float)totalDeviation/(float)getTimeStep();
}

void SUMOConnector::incrementTotalDeviation(size_t inc) {
  totalDeviation += inc;
}
