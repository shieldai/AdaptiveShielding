#include <fstream>
#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <cassert>

#include "Shield.h"
#include "STORMConnector.h"
#include "Util.h"

Shield::Shield(const std::string &tlsID, const Environment &environment, const Controller &controller)
    : ShieldModelGenerator(tlsID), tlsID(tlsID), active(true),
      environment(environment), controller(controller),
      strategy(tlsID, environment.getStateSpaceLabels()) {
  setFilename(out_path_ + tlsID + ".json");

  // default model
  setModelType("mdp");
  std::vector<std::string> module;
  module.push_back("Rmin=? [ LRA ]");
  setProperties(module);

  createPRISMArbiter(controller);
  createPRISMController(controller);
  createPRISMShield(controller);
  createPRISMRewards(controller);
  createPRISMEnvironment(environment);

  environment.check();
  controller.check();

  lastStateSpace = this->environment.getStateSpace();
  lastEnvironmentProbabilities = this->environment.getProbabilities();
}

bool Shield::state() const {
  return active;
}

void Shield::disable() {
  active = false;
}

std::string Shield::getJunction() const {
  return tlsID;
}

void Shield::setEnvironment(Environment &newEnvironment) {
  this->environment = std::move(newEnvironment);
}

const Environment *Shield::getEnvironment() const {
  return &environment;
}

void Shield::setController(Controller &newController) {
  this->controller = std::move(newController);
}

const Controller *Shield::getController() const {
  return &controller;
}

float Shield::getStateProbabilitiesDelta() const {
  return probDelta;
}

int Shield::getShieldGeneration() const {
  return generation;
}

void Shield::printConfig() {
  std::cout << "PRINT Config:\n";
  std::cout << tlsID << std::endl;

  std::cout << "\t" << environment.getStateSpaceString() << std::endl;
  std::cout << "\t" << controller.getActionSpaceString() << std::endl;
}

std::string Shield::logConfig() {
  std::stringstream log("");
  log << std::setprecision(2);
  log << tlsID << ": ";

  log << "\t" << environment.getStateSpaceString() << std::endl;
  log << "\t" << controller.getActionSpaceString() << std::endl;

  return log.str();
}

void Shield::createStrategy() {

  createPRISMFile(environment, controller);
  createPropFile();

  STORMConnector::instance().startStrategyUpdate(this);
}

Strategy *Shield::getStrategy() {
  return &strategy;
}

void Shield::updateStrategyCallback() {
  getStrategy()->loadSchedFile();
  getStrategy()->exportStrategy();

  generation++;

  // update state space
  if(!newStateSpace.empty()) {
    lastStateSpace = environment.getStateSpace();
  }
}

void Shield::lockStateSpaceSize() {
  if(ALLOW_FAIL_ON_STATE_SPACE_SIZE) {
    assert("STORM died due to the PRISM complexity, check out ALLOW_FAIL_ON_STATESPACE_SIZE");
  }

  // if STORM crash due to the complexity we lock the state space on the size before the crash
  maxStateSpaceSizeReached = true;
  environment.setStateSpace(lastStateSpace); // reset to working size
}

std::vector<int> Shield::checkStateInfo() {
  std::vector<int> newStateInfo(stateSpaceHistory.at(0).size(), 0);

  for(const auto &h : stateSpaceHistory) {
    for(size_t i = 0; i < newStateInfo.size(); i++) {
      int update = h.at(i) + 1;
      if(newStateInfo.at(i) < update) {
        newStateInfo.at(i) = update;
      }
    }
  }

  return newStateInfo;
}

void Shield::updateStateSpace() {
  //std::lock_guard<std::mutex> lock{mutexStrategy};
  environment.updateStateSpace(checkStateInfo());
}

void Shield::updateStateProbabilities() {
  environment.updateProbabilities();
}

void Shield::updateActionProbabilities() {
  controller.updateProbabilities();
}

void Shield::updateHaltingNumbers(const std::vector<int> &haltingNumbers) {
  environment.updateHaltingNumbers(haltingNumbers);
}

void Shield::updateVehicleNumbers(const std::vector<int> &vehicleNumbers) {
  environment.updateVehicleNumbers(vehicleNumbers);
}

void Shield::updateJunctionPhase(int currentJunctionPhase) {
  controller.updateJunctionPhase(currentJunctionPhase);
}

void Shield::update() {
  clock_t start = clock();

  std::vector<float> environmentProbabilities = environment.getProbabilities();
  std::vector<int> stateSpace = environment.getStateSpace();

  // NOTE currently only static updates
  bool doUpdate = gConfig.staticUpdate;

  //updateControllerProbabilities();
  updateStateProbabilities();
  updateStateSpace();

  std::vector<float> currentEnvironmentProbabilities = environment.getProbabilities();
  std::vector<int> currentStateSpace = environment.getStateSpace();

  probDelta = 0.;
  for(size_t i = 0; i < currentEnvironmentProbabilities.size(); i++) {
    probDelta += std::abs(currentEnvironmentProbabilities[i] - environmentProbabilities[i]);
  }

  stateDelta = 0;
  for(size_t i = 0; i < currentStateSpace.size(); i++) {
    stateDelta += std::abs(currentStateSpace[i] - stateSpace[i]);
  }

  if(!lastEnvironmentProbabilities.empty()) {
    assert(lastEnvironmentProbabilities.size()==currentEnvironmentProbabilities.size());
    float sumDelta = 0.;
    for(size_t i = 0; i < currentEnvironmentProbabilities.size(); i++) {
      sumDelta += std::abs(currentEnvironmentProbabilities[i] - lastEnvironmentProbabilities[i]);
    }

    if(sumDelta > UPDATE_PROBABILITY_DELTA) {
      doUpdate = true;
    }
  }

  // state space update can crash STORM
  if(!maxStateSpaceSizeReached) {
    if(!lastStateSpace.empty() && !doUpdate) {
      assert(lastStateSpace.size()==currentStateSpace.size());
      for(size_t i = 0; i < currentStateSpace.size(); i++) {
        auto delta = std::abs(currentStateSpace[i] - lastStateSpace[i]);
        if(delta > UPDATE_STATE_DELTA) {
          doUpdate = true;
          break;
        }
      }
    }
  }

  if(doUpdate) {
    // std::cout << "DO Update for " << junction << std::endl;
    lastEnvironmentProbabilities = currentEnvironmentProbabilities;

    //writeJson();
    createStrategy();
  }

  // std::cout << "Update success after " << float(clock() - start)/CLOCKS_PER_SEC << "s!" << std::endl;
}



int Shield::getNextAction(int currentAction) {
  int shieldAction = -1;

  // Clip current state space on SUMO tls with state space from environment/shield,
  // which states the max. state space from the current Strategy.
  auto currentStateSpace = environment.getHaltingNumbers();
  auto shieldStateSpace = environment.getStateSpace();
  assert(currentStateSpace.size()==shieldStateSpace.size());
  for(size_t i = 0; i < currentStateSpace.size(); i++) {
    if(currentStateSpace.at(i) > shieldStateSpace.at(i)) {
      currentStateSpace.at(i) = shieldStateSpace.at(i);
    }
  }

  try {
    if(!strategy.check()) {
      return shieldAction;
    }

    stateSpaceHistory.push_back(currentStateSpace);
    shieldAction = getStrategy()->getStrategyAction(currentStateSpace, currentAction);
  } catch(std::exception &e) {
    std::cerr << " -> " << environment.getStateSpaceSizeString() << "!" << std::endl;
  }

  return shieldAction;
}

void Shield::readJson() {
  ShieldConfig::readJson();

  active = parseState();
  auto junctionJson = parseSetting();
  assert(tlsID==junctionJson && "");

  tlsID = junctionJson;

  setModelType(parseModelType());
  setProperties(parseProperties());

  setPRISMArbiter(parseModulesArbiter());
  setPRISMEnvironment(parseModulesEnvironment());
  setPRISMController(parseModulesController());
  setPRISMShield(parseModulesShield());
  setPRISMRewards(parseModulesRewards());

  auto environmentJson = parseEnvironment();
  if(environment.check()) {
    environment.updateProperties(environmentJson);
  } else {
    environment = environmentJson;
  }

  auto controllerJson = parseController();
  if(controller.check()) {
    controller.updateProperties(controllerJson);
  } else {
    controller = controllerJson;
  }
}

void Shield::writeJson() {
  storeModelType(modelType_);
  storeSetting(tlsID, active);
  storeProperties(properties);
  storeModulesArbiter(module_arbiter);
  storeModulesEnvironment(module_environment);
  storeModulesController(module_controller);
  storeModulesShield(module_shield);
  storeModulesRewards(module_rewards);
  storeController(controller);
  storeEnvironment(environment);

  ShieldConfig::writeJson();
}

Shield *buildFromFile(const std::string &shieldConfigFile) {
  if(fileExist(shieldConfigFile)) {
    std::cerr << "Error: " << strerror(errno);
    exit(1);
  }

  auto config = ShieldConfig(shieldConfigFile);
  auto s = new Shield(config.parseSetting(), config.parseEnvironment(), config.parseController());
  s->setFilename(shieldConfigFile);

  return s;
}
