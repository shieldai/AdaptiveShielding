#include <cmath>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <fstream>

#include "Strategy.h"
#include "Util.h"

Strategy::Strategy(const std::string &filePrefix, const std::vector<std::string> &labels)
    : labels(labels), filePrefix(filePrefix) {
  assert(check());
}

bool Strategy::check() const {
  if(!labels.empty() &&
      !filePrefix.empty()) {
    return true;
  }

  return false;
}

void Strategy::addStrategyStep(const std::vector<int> &state, int currentAction, int nextAction) {
  strategy_[std::make_pair(state, currentAction)] = nextAction;
}

int Strategy::getStrategyAction(std::pair<std::vector<int>, int> simulationState) {

  // not interesting
  if(std::all_of(simulationState.first.begin(), simulationState.first.end(), [](int i) { return i==0; })) {
    return -1;
  }

  //for(int i : state.first)
  //  std::cout << std::to_string(i) + ",";
  //std::cout << ";" << std::to_string(state.second) << std::endl;

  try {
    return strategy_.at(simulationState);
  } catch(std::exception &e) {
    std::cerr << e.what() << " No shield action available!" << std::endl;

    // FIND BEST MATCH
    for(size_t i = 0; i < simulationState.first.size(); i++) {
      std::pair<std::vector<int>, int> testState = simulationState;
      if(testState.first[i]!=0) {
        testState.first[i]--;
        try {
          return strategy_.at(testState);
        } catch(std::exception &e) {
          continue;
        }
      }
    }

    throw;
  }
}

int Strategy::getStrategyAction(const std::vector<int> &state, int currentAction) {
  return getStrategyAction(std::make_pair(state, currentAction));
}

void Strategy::exportStrategy() {
  std::ofstream stratFile(out_path_ + filePrefix + ".strat");
  stratFile << "// " << filePrefix + ".strat" << " Created at " << getTimeString() << std::endl;

  for(std::pair<std::pair<std::vector<int>, int>, int> mapping : strategy_) {
    std::string line;
    for(auto state : mapping.first.first) {
      line += std::to_string(state) + ",";
    }

    line.back() = ';';
    line += std::to_string(mapping.first.second) + " -> " + std::to_string(mapping.second) + "\n";
    stratFile << line;
  }
  stratFile.close();
}

void Strategy::loadSchedFile() {
  assert(check());

  std::ifstream schedFile;
  schedFile.open(out_path_ + filePrefix + ".sched");
  std::string line;

  std::vector<int> state;
  int currentAction = -1;
  int nextAction = -1;

  strategy_.clear();
  while(std::getline(schedFile, line)) {
    if(parseSchedFileLine(line, state, currentAction, nextAction)) {
      addStrategyStep(state, currentAction, nextAction);
    }
  }

  schedFile.close();
}

bool Strategy::parseSchedFileLine(const std::string &line,
                                  std::vector<int> &state,
                                  int &currentAction,
                                  int &nextAction) {

  if(getValueForToken(line, "move")!="2") {
    return false;
  }

  state.clear();
  for(const auto &label : labels) {
    state.push_back(std::stoi(getValueForToken(line, label)));
  }

  std::string action = getValueForToken(line, "action");
  int actionID = std::stoi(action);
  currentAction = actionID;

  std::string actionLabel = getSubstrBetweenDelims(line, "{", "}");

  std::string actionLabelID;
  for(char c : actionLabel) {
    if(std::isdigit(c)) {
      actionLabelID += c;
    }
  }

  nextAction = std::stoi(actionLabelID);
  return true;
}
