#include <iostream>
#include <fstream>

#include "ShieldConfig.h"
#include "Environment.h"
#include "Controller.h"
#include "Util.h"

ShieldConfig::ShieldConfig(const std::string &filename) : configFilename(filename) {
  if(fileExist(filename)) {
    readJson();
  }
}

void ShieldConfig::setFilename(const std::string &filename) {
  configFilename = filename;
}

const std::string &ShieldConfig::getFilename() const {
  return configFilename;
}

void ShieldConfig::writeJson() {
  std::ofstream o(configFilename, std::ios::trunc);
  o << config.dump(2);
}

void ShieldConfig::readJson() {
  std::ifstream i(configFilename);
  i >> config;
}

bool ShieldConfig::parseState() {
  return config.at("active").get<bool>();
}

std::string ShieldConfig::parseModelType() {
  return config.at("modelType").get<std::string>();
}

std::string ShieldConfig::parseSetting() {
  return config.at("setting").get<std::string>();
}

std::vector<std::string> ShieldConfig::parseProperties() {
  return config.at("properties").get<std::vector<std::string>>();
}

std::vector<std::string> ShieldConfig::parseModulesArbiter() {
  return config.at("model").at("modules").at("arbiter").at("moves").get<std::vector<std::string>>();
}

std::vector<std::string> ShieldConfig::parseModulesEnvironment() {
  return config.at("model").at("modules").at("environment").at("moves").get<std::vector<std::string>>();
}

std::vector<std::string> ShieldConfig::parseModulesController() {
  return config.at("model").at("modules").at("controller").at("moves").get<std::vector<std::string>>();
}

std::vector<std::string> ShieldConfig::parseModulesShield() {
  return config.at("model").at("modules").at("shield").at("moves").get<std::vector<std::string>>();
}

std::vector<std::string> ShieldConfig::parseModulesRewards() {
  return config.at("model").at("rewards").get<std::vector<std::string>>();
}

Controller ShieldConfig::parseController() {
  auto actions = config.at("model").at("global").at("controller").at("actions").get<std::vector<std::string>>();
  auto probabilities = config.at("model").at("global").at("controller").at("probabilities").get<std::vector<float>>();
  auto ways = config.at("model").at("global").at("controller").at("ways").get<std::vector<std::vector<std::string>>>();

  return Controller(actions, probabilities, ways);
}

Environment ShieldConfig::parseEnvironment() {
  auto labels = config.at("model").at("global").at("environment").at("labels").get<std::vector<std::string>>();
  auto probabilities = config.at("model").at("global").at("environment").at("probabilities").get<std::vector<float>>();
  auto weights = config.at("model").at("global").at("environment").at("weights").get<std::vector<int>>();
  auto max = config.at("model").at("global").at("environment").at("stateSpace").get<std::vector<int>>();

  if(RESET_JSON_DIST) {
    return Environment(labels, weights);
  }

  return Environment(labels, probabilities, weights, max);
}

void ShieldConfig::storeModelType(std::string modelType) {
  config["modelType"] = modelType;
}

void ShieldConfig::storeSetting(std::string tlsID, bool state) {
  config["active"] = state;
  config["setting"] = tlsID;
}

void ShieldConfig::storeProperties(std::vector<std::string> module) {
  config["properties"] = module;
}

void ShieldConfig::storeModulesArbiter(std::vector<std::string> module) {
  config["model"]["modules"]["arbiter"]["moves"] = module;
}

void ShieldConfig::storeModulesEnvironment(std::vector<std::string> module) {
  config["model"]["modules"]["environment"]["moves"] = module;
}

void ShieldConfig::storeModulesController(std::vector<std::string> module) {
  config["model"]["modules"]["controller"]["moves"] = module;
}

void ShieldConfig::storeModulesShield(std::vector<std::string> module) {
  config["model"]["modules"]["shield"]["moves"] = module;
}

void ShieldConfig::storeModulesRewards(std::vector<std::string> module) {
  config["model"]["rewards"] = module;
}

void ShieldConfig::storeController(Controller &controller) {
  config["model"]["global"]["controller"]["actions"] = controller.getActionSpace();
  config["model"]["global"]["controller"]["probabilities"] = controller.getProbabilities();
  config["model"]["global"]["controller"]["ways"] = controller.getActionSpaceWays();
}

void ShieldConfig::storeEnvironment(Environment &environment) {
  config["model"]["global"]["environment"]["labels"] = environment.getStateSpaceLabels();
  config["model"]["global"]["environment"]["probabilities"] = environment.getProbabilities();
  config["model"]["global"]["environment"]["weights"] = environment.getWeights();
  config["model"]["global"]["environment"]["stateSpace"] = environment.getStateSpace();

}

