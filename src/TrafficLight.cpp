#include <cassert>
#include <algorithm>
#include <set>
#include <sstream>

#include "TrafficLight.h"

#include "SUMOConnector.h"
#include "Shield.h"
#include "Util.h"

TrafficLight *TrafficLight::build(SUMOConnector *sumo, const std::string &tlsID) {
  auto *t = new TrafficLight(sumo, tlsID);
  assert(t->getShield()!=nullptr);

  t->getShield()->writeJson();
  t->getShield()->createStrategy();
  t->getShield()->printConfig();
  t->logHeader();

  return t;
}

TrafficLight *TrafficLight::buildFromFile(SUMOConnector *sumo, const std::string &shieldConfigFile) {
  if(fileExist(shieldConfigFile)) {
    std::cerr << "Error: " << strerror(errno);
    exit(1);
  }

  std::string tlsID = ShieldConfig(shieldConfigFile).parseSetting();
  auto *t = new TrafficLight(sumo, tlsID);
  assert(t->getShield()!=nullptr);

  t->getShield()->setFilename(shieldConfigFile);
  t->getShield()->readJson();
  // write the config back if some orders are corrected.
  t->getShield()->writeJson();
  t->getShield()->createStrategy();
  t->getShield()->printConfig();
  t->logHeader();

  return t;
}

TrafficLight::TrafficLight(SUMOConnector *sumo, const std::string &tls_id)
    : sumo_(sumo), shield_(nullptr), tlsID(tls_id),
      laneMapper(sumo, tls_id),
      phaseMapper(sumo, tls_id, laneMapper.getFormattedLinks()),
      iconManager(sumo, tls_id),
      log(gConfig.logFile + "." + tls_id, std::ofstream::out | std::ofstream::trunc) {

  assert("We do not support dynamic traffic lights."
             && libsumo::TRAFFICLIGHT_TYPE_STATIC==sumo->getTrafficLightsAllProgramLogics(tlsID)[0].type);

  clock_t start = clock();
  auto tlsIDs = sumo->getTrafficLightIDs();

  //Check tls ID
  if(std::find(tlsIDs.begin(), tlsIDs.end(), tlsID)==tlsIDs.end()) {
    std::cerr << "Error: Invalid Traffic Light ID " << tlsID;
    exit(1);
  }

  for(auto &lane : sumo->getTrafficLightsControlledLanes(tlsID)) {
    formatName(lane);
    if(std::find(controlledLanes.begin(), controlledLanes.end(), lane)==controlledLanes.end()) {
      controlledLanes.push_back(lane);
    }
  }

  auto labels = laneMapper.getLabels();

  // NOTE: weights are currently unused
  std::vector<int> weights(labels.size(), 1);
  assert(labels.size()==weights.size());

  /*
  log << "// Created at " << getTimeString() << "\n\n";
  for(size_t i = 0; i < labels.size(); i++) {
    log << "STORM Label " << labels[i] << " [" << weights[i] << "]" << std::endl;
    log << "\tSUMO Labels: ";
    for(const auto &sl : laneMapper.getSumoLabels(labels[i]))
      log << sl << ", ";
    log << std::endl;
  }

  log << "-------------------------------------------------------------------------\n";
  */

  if(labels.size() < 2) {
    // shield is not enable
    std::cerr << "Only one controlled lane! - Not supported in shield pattern." << std::endl;
    return;
  }

  assert(!labels.empty());
  auto environment = Environment(labels, weights);

  assert(phaseMapper.getPhases().size() >= 2);
  auto controller = Controller(phaseMapper.getPhases());

  shield_ = new Shield(tlsID, environment, controller);

  std::cout << "Load Traffic Light data " << float(clock() - start)/CLOCKS_PER_SEC << "s!" << std::endl;
}

TrafficLight::~TrafficLight() {
  if(shield_!=nullptr) {
    delete shield_;
    shield_ = nullptr;
  }
}

bool TrafficLight::getShieldState() const {
  return activeShield;
}

std::string TrafficLight::getTrafficLightID() {
  return tlsID;
}

int TrafficLight::getJunctionPhase() {
  return shield_->getController()->getJunctionPhase();
}

int TrafficLight::getNextPhaseID() {
  auto currentJunctionPhase = getJunctionPhase();
  int shieldAction = shield_->getNextAction(currentJunctionPhase);
  return shieldAction;
}

Shield *TrafficLight::getShield() {
  return shield_;
}

void TrafficLight::step() {
  if(getShield()==nullptr || !getShield()->state()) {
    return;
  }

  iconManager.step();
  track();

  auto action = getJunctionPhase();
  bool shieldUpdated = false;
  int deviation = 0;

  size_t timeStep = sumo_->getTimeStep();
  if(timeStep > warmUpTime && timeStep%updateInterval==0 && sumo_->getVehicleIDs().size()) {
    getShield()->update();
    std::cout << timeStep << ": " << getShield()->logConfig() << std::endl;
    shieldUpdated = true;
  }

  if (gConfig.overwrite) {

    /// NOTE: If we have the RL Agent/controller in the system we can not restore with the static controller
    /// Therefore, we just overwrite the RL Agent.

    if(timeStep%STEP_IN_DELTA==0) {
      if (lastOverwrittenAction != -1 && lastShieldAction == phaseMapper.getPhaseID()) {
        phaseMapper.setPhaseID(lastOverwrittenAction);
        //std::cerr << "(" << sumo_->getTimeStep() << ") RESET TO ACTION ("
        //  << lastOverwrittenAction << ") " << lastOverwrittenAction*2 << std::endl;
      }

      activeShield = false;
      lastOverwrittenAction = -1;

      action = phaseMapper.getControllerPhase();
      auto shieldAction = getShield()->getNextAction(action);

      if(shieldAction!=-1) {
        if(shieldAction!=action) {
          deviation = 1;

          sumo_->incrementTotalDeviation(deviation);

          trackShield(shieldAction);
          trackInterference(deviation);

          iconManager.showIcon();
          //std::cerr << "(" << sumo_->getTimeStep() << ") SHIELD ACTION (" << shieldAction << ") " << shieldAction*2 << std::endl;

          lastOverwrittenAction = phaseMapper.getPhaseID();
          lastShieldAction = shieldAction;

          phaseMapper.setPhaseID(shieldAction);

          activeShield = true;

          action = shieldAction;
        }
      }
    } else if(getShieldState()) {
      // no shield interference but log deviation, if strategy is enable
      deviation = 1;

      if(phaseMapper.getPhaseID()!=lastShieldAction)
        phaseMapper.setPhaseID(lastShieldAction);
    }
  }
  else {
    if(timeStep%STEP_IN_DELTA==0) {
      phaseMapper.restoreController();
      activeShield = false;

      action = phaseMapper.getControllerPhase();
      auto shieldAction = getShield()->getNextAction(action);

      if(shieldAction!=-1) {
        if(shieldAction!=action) {
          deviation = 1;

          sumo_->incrementTotalDeviation(deviation);

          trackShield(shieldAction);
          trackInterference(deviation);

          iconManager.showIcon();
          phaseMapper.setPhaseID(shieldAction);
          activeShield = true;

          action = shieldAction;
        }
      }
    } else if(getShieldState()) {
      // no shield interference but log deviation, if strategy is enable
      deviation = 1;
    }
  }

  track2(shieldUpdated, deviation, action);

  // avoid the change in another program
  // if(getShield()!=nullptr && getShield()->state()) {
  //   resetProgram();
  // }
}

void TrafficLight::track() {
  if(shield_==nullptr || !shield_->state()) {
    return;
  }

  std::vector<int> lastStepVehicleNumbers;
  std::vector<int> lastStepHaltingNumbers;

  for(const auto &lane : shield_->getEnvironment()->getStateSpaceLabels()) {
    int lastStepVehicleNumber = 0;
    int lastStepHaltingNumber = 0;

    if(gConfig.noTrees) {
      for(auto lane : laneMapper.getSumoLabel(lane)) {
        int vn = sumo_->getLaneLastStepVehicleNumber(lane);
        int hn = sumo_->getLaneLastStepHaltingNumber(lane);
        lastStepVehicleNumber = std::max(vn, lastStepVehicleNumber);
        lastStepHaltingNumber = std::max(hn, lastStepHaltingNumber);
      }
    } else {
      // FEATURE ACCURATE LANE TRACKING, REAL WORLD(IMPORTED MAPS
      for(auto tree : laneMapper.getSumoLabelTree(lane)) {
        int vn = 0;
        int hn = 0;

        tree->trackNode(vn, hn);
        lastStepVehicleNumber = std::max(vn, lastStepVehicleNumber);
        lastStepHaltingNumber = std::max(hn, lastStepHaltingNumber);
      }
    }

    lastStepHaltingNumbers.push_back(lastStepHaltingNumber);
    lastStepVehicleNumbers.push_back(lastStepVehicleNumber);
  }

  shield_->updateVehicleNumbers(lastStepVehicleNumbers);
  shield_->updateHaltingNumbers(lastStepHaltingNumbers);

  auto currentJunctionPhase = phaseMapper.getPhaseID();

  // THIS CONTROLLER PHASES !!!
  phaseMapper.step();

  // CHECK IF INTERNAL STRUCT SYNCS WITH CONTROLLER
  //assert(currentJunctionPhase == controllerPhase);
  //assert(sumo_->trafficlights.getPhase(tlsID) == controllerSumoPhase);
  //assert(sumo_->trafficlights.getNextSwitch(tlsID) - sumo_->simulation.getTime() == sumoPhaseDurationCountdown);

  shield_->updateJunctionPhase(currentJunctionPhase);

  log << sumo_->getTime() << ",";

  std::string out = "[";
  for(const auto &hn : lastStepHaltingNumbers)
    out += std::to_string(hn) + " ";
  out.back() = ']';
  log << out << ",";
}

void TrafficLight::track2(int shieldUpdated, int dev, int action) {

  if(shield_==nullptr || !shield_->state())
    return;

  // takes information before manipulation
  auto currentJunctionPhase = phaseMapper.getPhaseID();

  std::string out = "[";
  for(const auto &s : shield_->getEnvironment()->getStateSpace())
    out += std::to_string(s) + " ";
  out.back() = ']';
  log << out << ",";

  out = "[";
  for(const auto &p : shield_->getEnvironment()->getProbabilities())
    out += std::to_string(p) + " ";
  out.back() = ']';
  log << out << ",";

  log << shield_->getStateProbabilitiesDelta() << ",";
  log << shield_->getShieldGeneration() << ",";
  log << shieldUpdated << ",";
  log << phaseMapper.getControllerPhase() << ",";
  log << currentJunctionPhase << ",";
  // action after reset/correction/shield
  log << action << ",";
  log << dev << ",";
  log << "\n";
  log.flush();
}

void TrafficLight::logHeader() {

  if(shield_==nullptr || !shield_->state())
    return;

  log << "# ";
  log << "time,";
  log << "lastStepHaltingNumbers,";
  log << "StateSpace,";
  //log << "stateDelta,";
  log << "Probabilities,";
  log << "probDelta,";
  log << "generation,";
  log << "shieldUpdated,";
  log << "controllerPhase,";
  log << "currentJunctionPhase,";
  log << "junctionPhaseAfterShield,";
  log << "dev,";
  log << "\n";

  log << "# STATE SPACE LABELS: ";
  std::string out = "[";
  for(const auto &s : shield_->getEnvironment()->getStateSpaceLabels())
    out += s + ",";
  out.back() = ']';
  log << out << std::endl;

}

void TrafficLight::trackShield(int shieldAction) {
  shieldTracking_.push_back(shieldAction);
}

float TrafficLight::trackInterference(int interference) {
  interferenceCount_ += interference;
  interferenceRate_ = (float)interferenceCount_/(float)shieldTracking_.size();
  return interferenceRate_;
}

std::string TrafficLight::printEnvironmentState() {
  std::string str = shield_->getEnvironment()->getStateSpaceString();
  str += ":" + std::to_string(getJunctionPhase()) + "\n";
  return str;
}

std::string TrafficLight::logTracker() {
  std::stringstream log("");
  log << std::setprecision(2);
  log << tlsID << "\n";
  log << "controlledLanes: ";
  for(const auto &lane : controlledLanes)
    log << lane << " ";
  log << std::endl;
  for(uint i = 0; i < shield_->getController()->getActionSpace().size(); i++)
    log << i << ":" << shield_->getController()->getActionSpace().at(i) << " ";
  log << std::endl;
  for(const auto &sub : shield_->getEnvironment()->getStateSpaceLabels())
    log << sub << std::endl;
  log << std::endl;
  return log.str();
}

