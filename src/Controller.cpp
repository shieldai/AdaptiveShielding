#include <algorithm>
#include <iostream>
#include <cmath>
#include <cassert>

#include "Controller.h"

Controller::Controller(const std::vector<std::string> &actions,
                       const std::vector<float> &probabilities,
                       const std::vector<std::vector<std::string>> &ways)
    : actions(actions), probabilities(probabilities), ways(ways) {
  for(size_t i = 0; i < actions.size(); i++) {
    actionLabels.push_back("action" + std::to_string(i));
  }
  junctionPhaseStats.resize(actions.size(), 0);
}

Controller::Controller(const std::vector<struct phaseInfo> &phases) {
  float actionProb = 1.f/(float)phases.size();
  for(size_t i = 0; i < phases.size(); i++) {
    assert(phases[i].phaseID==(int)i);
    actions.push_back(phases[i].state);
    actionLabels.push_back("action" + std::to_string(i));
    probabilities.push_back(actionProb);
    ways.push_back(phases.at(i).activeInLanes);
    junctionPhaseStats.push_back(0);
  }

  assert(check());
}

bool Controller::check() const {
  if(actions.empty()) {
    return false;
  }

  if(actions.size()==actionLabels.size() &&
      actions.size()==probabilities.size() &&
      actions.size()==ways.size()) {
    return true;
  }
  return false;
}

void Controller::updateProbabilities() {
  updateProbabilities(getPMF());
}

void Controller::updateActionSpace(const std::vector<std::string> &actionSpace) {
  assert(actionSpace.size()==this->actions.size());
  this->actions = actionSpace;
}

void Controller::updateJunctionPhase(int currentJunctionPhase) {
  this->currentJunctionPhase = currentJunctionPhase;
  this->junctionPhaseStats.at(currentJunctionPhase)++;
}

void Controller::updateProperties(Controller &controller) {
  assert(actions.size()==controller.actions.size());
  assert(ways.size()==controller.ways.size());

  std::vector<float> newProbabilities;

  // ways must match!
  for(size_t i = 0; i < ways.size(); i++) {
    auto w1 = ways[i];
    sort(w1.begin(), w1.end());

    for(size_t j = 0; j < controller.ways.size(); j++) {
      auto w2 = controller.ways[j];
      sort(w2.begin(), w2.end());

      if(w1.size()==w2.size() && equal(w1.begin(), w1.end(), w2.begin())) {
        newProbabilities.push_back(controller.probabilities[j]);
      }
    }
  }

  assert(probabilities.size()==newProbabilities.size());
  probabilities = newProbabilities;
}

std::vector<std::string> Controller::getActionSpace() const {
  return actions;
}

std::vector<std::string> Controller::getActionSpaceLabels() const {
  return actionLabels;
}

std::vector<float> Controller::getProbabilities() const {
  return probabilities;
}

std::vector<std::vector<std::string>> Controller::getActionSpaceWays() const {
  return ways;
}

int Controller::getJunctionPhase() const {
  assert(currentJunctionPhase!=-1 && "ERROR: Use before update!");
  return currentJunctionPhase;
}

std::string Controller::getActionSpaceString() {
  std::string out = "ActionSpace = ";
  for(uint i = 0; i < actions.size(); i++) {
    out += actions.at(i) + ":" + std::to_string(probabilities.at(i));
    out += i!=actions.size() - 1 ? ", " : "";
  }

  return out;
}

void Controller::updateProbabilities(const std::vector<float> &probabilities) {
  if(probabilities.size()!=this->probabilities.size()) {
    std::cerr << "ERROR: parameter do not matches the current dimension of controller probabilities." << std::endl;
    return;
  }
  if(std::any_of(probabilities.begin(), probabilities.end(), [](float p) { return std::isnan(p); })) {
    std::cerr << "ERROR: Controller::updateProbabilities parameter contains invalid elements." << std::endl;
    return;
  }

  this->probabilities = probabilities;
  assert(isPMF(this->probabilities) && "controller probability is not a PMF!");
}

std::vector<float> Controller::getPMF() {
  return calculatePMF(junctionPhaseStats);
}

