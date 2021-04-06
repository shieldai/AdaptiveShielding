#include <algorithm>
#include <cassert>

#include "PhaseMapper.h"
#include "SUMOConnector.h"

PhaseMapper::PhaseMapper(ISumo *sumo,
                         const std::string &tls_id,
                         const std::vector<std::vector<std::string>> &links)
    : sumo(sumo), tlsID(tls_id) {

  //auto programLogics = sumo->trafficlights.getAllProgramLogics(tlsID);
  auto programLogics = sumo->getTrafficLightsAllProgramLogics(tlsID);
  auto tlsLogic = programLogics.at(0);

  int phaseID = -1;
  for(int sumoPhaseID = 0; sumoPhaseID < (int)tlsLogic.phases.size(); sumoPhaseID++) {
    // take only num_phases without yellow!
    auto state = tlsLogic.phases.at(sumoPhaseID)->state;
    //assert(state.size()==links.size());

    if(state.find('y')==std::string::npos) {
      phaseID++;

      struct phaseInfo info;
      info.phaseID = phaseID;
      info.sumoPhaseID = sumoPhaseID;
      info.state = state;
      info.activeInLanes = getActiveLaneLabels(links, state);
      phases.push_back(info);

      this->sumoPhaseID.push_back(sumoPhaseID);
    }

    if(phaseID < 0) {
      throw std::logic_error("Not implemented, we assumed that the program start with non-yellow phase!");
    }

    sumoPhaseIDtoPhaseID.insert(std::pair<int, int>(sumoPhaseID, phaseID));
  }

  for(const auto &pair : sumoPhaseIDtoPhaseID) {
    auto d = programLogics[0].phases[pair.first]->duration;
    phaseIDtoDuration[pair.second] += d;
    sumoPhaseIDtoDuration[pair.first] = d;
  }

  // SET PHASE TO 0, BE SURE WE START FROM THE SAME POINT
  sumo->setTrafficLightPhase(tlsID, 0);
  sumo->setTrafficLightProgram(tlsID, "0");

  controllerPhase = 0;
  controllerSumoPhase = 0;

  phaseDurationCountdown = phaseIDtoDuration[controllerPhase];
  sumoPhaseDurationCountdown = sumoPhaseIDtoDuration[controllerSumoPhase];
}

void PhaseMapper::step() {

  if(phaseDurationCountdown==0) {
    controllerPhase = (controllerPhase + 1)%phaseIDtoDuration.size();
    phaseDurationCountdown = phaseIDtoDuration[controllerPhase];
  }
  phaseDurationCountdown--;

  if(sumoPhaseDurationCountdown==0) {
    controllerSumoPhase = (controllerSumoPhase + 1)%sumoPhaseIDtoDuration.size();
    sumoPhaseDurationCountdown = sumoPhaseIDtoDuration[controllerSumoPhase];
  }
  sumoPhaseDurationCountdown--;
}

std::vector<std::string> PhaseMapper::getActiveLaneLabels(
    const std::vector<std::vector<std::string>> &links,
    const std::string &state) {
  assert(state.size()==links.size());
  std::vector<std::string> activeLanes;

  for(size_t i = 0; i < state.size(); i++) {
    char s = state[i];
    if(s=='G' || s=='g' /* small g has no priority! - wat happens if we ignore it? */) {
      auto lanes = links.at(i);
      activeLanes.insert(activeLanes.end(), lanes.begin(), lanes.end());
    }
  }

  auto set_ = std::set<std::string>(activeLanes.begin(), activeLanes.end());
  // ADD VIRT LANE
  for(const auto &laneID : set_) {
    if(sumo->hasVirtualLane(laneID)) {
      set_.insert(sumo->virtualLaneID(laneID));
    }
  }

  return std::vector<std::string>(set_.begin(), set_.end());
}

const std::vector<struct phaseInfo> &PhaseMapper::getPhases() {
  return phases;
}

int PhaseMapper::getControllerPhase() const {
  return controllerPhase;
}

int PhaseMapper::getSumoPhaseID(int phaseID) const {
  assert(std::find(sumoPhaseID.begin(), sumoPhaseID.end(), -1)==sumoPhaseID.end() && "ERROR: Use before update!");
  return sumoPhaseID.at(phaseID);
}

int PhaseMapper::getPhaseID() const {
  auto sumoPhaseID = sumo->getTrafficLightCurrentPhase(tlsID);
  assert(sumoPhaseID!=-1);
  auto phaseID = sumoPhaseIDtoPhaseID.at(sumoPhaseID);
  assert(phaseID!=-1);
  return phaseID;
}

void PhaseMapper::setPhaseID(const int phaseID) const {
  auto sumoPhaseID = getSumoPhaseID(phaseID);
  sumo->setTrafficLightPhase(tlsID, sumoPhaseID);
}

void PhaseMapper::restoreController() {
  // RESET THIS CONTROLLER PHASE, ALSO WHEN DURATION IS 0 TO GET BACK IN SYNC
  sumo->setTrafficLightPhase(tlsID, controllerSumoPhase);
  sumo->setTrafficLightPhaseDuration(tlsID, sumoPhaseDurationCountdown);
}

void PhaseMapper::resetProgram() {
  if(sumo->getTrafficLightCurrentProgram(tlsID)!="0")
    sumo->setTrafficLightProgram(tlsID, "0");
}


