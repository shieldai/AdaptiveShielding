#include <algorithm>
#include <iostream>
#include <cmath>
#include <cassert>

#include "Environment.h"

Environment::Environment(const std::vector<std::string> &labels,
                         const std::vector<float> &probabilities,
                         const std::vector<int> &weights,
                         const std::vector<int> &state_space)
    : labels(labels), probabilities(probabilities), weights(weights), stateSpace(state_space) {
  vehicleNumbers.resize(labels.size(), 0);
  allVehicleNumbers.resize(labels.size(), 0);
  allNewVehicleNumbers.resize(labels.size(), 0);
  haltingNumbers.resize(labels.size(), 0);
  environmentTracking_.resize(labels.size(), 0);
}

Environment::Environment(std::vector<std::string> &lanes, std::vector<int> &weights) {
  float stateProb = 1.f/(float)lanes.size();

  for(auto weight : weights) {
    weight = std::max(MIN_LANE_WEIGHT, weight);
    weight = std::min(MAX_LANE_WEIGHT, weight);
    this->weights.push_back(weight);
  }

  for(const auto &lane : lanes) {
    labels.push_back(lane);
    probabilities.push_back(stateProb);
    weights.push_back(MIN_LANE_WEIGHT);
    stateSpace.push_back(MIN_LANE_SIZE);

    vehicleNumbers.push_back(0);
    haltingNumbers.push_back(0);
    allVehicleNumbers.push_back(0);
    allNewVehicleNumbers.push_back(0);
    environmentTracking_.push_back(0);
  }

  assert(check());
}

bool Environment::check() const {

  if(labels.empty())
    return false;

  if(labels.size()==probabilities.size() &&
      labels.size()==weights.size() &&
      labels.size()==stateSpace.size() &&
      labels.size()==vehicleNumbers.size() &&
      labels.size()==haltingNumbers.size() &&
      labels.size()==allVehicleNumbers.size() &&
      labels.size()==allNewVehicleNumbers.size() &&
      labels.size()==environmentTracking_.size() &&
      isPMF(probabilities)) {
    return true;
  }

  return false;
}

void Environment::updateWeights(const std::vector<int> &weights) {
  assert(this->weights.size()==weights.size());
  for(size_t i = 0; i < this->weights.size(); i++) {
    auto update = weights[i];
    if(update < 1) {
      update = 1;
    } else if(update > MAX_LANE_WEIGHT) {
      update = MAX_LANE_WEIGHT;
    }
    this->weights[i] = update;
  }
}

void Environment::updateProbabilities() {
  updateProbabilities(getPMF());
}

void Environment::updateStateSpace(const std::vector<int> &stateSpace) {
  assert(this->stateSpace.size()==stateSpace.size());
  for(size_t i = 0; i < this->stateSpace.size(); i++) {
    auto update = stateSpace[i];
    if(this->stateSpace[i] > update) {
      // std::cerr << "shrinking of state space not supported!" << std::endl;
      continue;
    }

    if(update < 1) {
      update = 1;
    } else if(update > gConfig.maxLaneSize) {
      update = gConfig.maxLaneSize;
    }

    this->stateSpace[i] = update;
  }
}

void Environment::setStateSpace(const std::vector<int> &stateSpace) {
  this->stateSpace = stateSpace;
}

void Environment::updateHaltingNumbers(const std::vector<int> &haltingNumbers) {
  assert(this->haltingNumbers.size()==haltingNumbers.size());
  for(size_t i = 0; i < haltingNumbers.size(); i++) {
    this->haltingNumbers[i] = haltingNumbers[i];
  }
}

void Environment::updateVehicleNumbers(const std::vector<int> &vehicleNumbers) {
  assert(this->vehicleNumbers.size()==vehicleNumbers.size());
  for(size_t i = 0; i < vehicleNumbers.size(); i++) {
    auto lastVehicleNumber = vehicleNumbers[i];
    this->vehicleNumbers[i] = vehicleNumbers[i];
    this->allVehicleNumbers[i] += vehicleNumbers[i];
    int delta = this->vehicleNumbers[i] - lastVehicleNumber;
    if(delta > 0)
      this->allNewVehicleNumbers[i] += delta;

    this->environmentTracking_[i] += vehicleNumbers[i];

  }
}

void Environment::updateProperties(Environment &environment) {
  auto l1 = labels;
  auto l2 = environment.labels;
  assert(l1.size()==l2.size());

  sort(l1.begin(), l1.end());
  sort(l2.begin(), l2.end());
  assert(equal(l1.begin(), l1.end(), l2.begin()));

  // make sure you get the information it the right label order!
  for(size_t i = 0; i < labels.size(); i++) {
    for(size_t j = 0; j < environment.labels.size(); j++) {
      if(labels[i]==environment.labels[j]) {
        probabilities[i] = environment.probabilities[j];
        weights[i] = environment.weights[j];
        stateSpace[i] = environment.stateSpace[j];
      }
    }
  }
}

std::vector<std::string> Environment::getStateSpaceLabels() const {
  return labels;
}

std::vector<int> Environment::getStateSpace() const {
  return stateSpace;
}

std::vector<int> Environment::getWeights() const {
  return weights;
}

std::vector<float> Environment::getProbabilities() const {
  return probabilities;
}

std::vector<int> Environment::getHaltingNumbers() {
  return haltingNumbers;
}

std::vector<int> Environment::getVehicleNumbers() {
  return vehicleNumbers;
}

std::string Environment::getStateSpaceString() const {
  std::string out = "StateSpace = ";
  for(uint i = 0; i < labels.size(); i++) {
    out += labels.at(i) + ": " + std::to_string(stateSpace.at(i))
        + " *" + std::to_string(weights.at(i))
        + " [" + std::to_string(probabilities.at(i)) + "]";
    out += i!=labels.size() - 1 ? " x " : "";
  }
  return out;
}

std::string Environment::getStateSpaceSizeString() const {
  std::string out = "State Space: ";
  for(uint i = 0; i < stateSpace.size(); i++) {
    out += std::to_string(stateSpace.at(i));
    out += i!=labels.size() - 1 ? "," : "";
  }
  return out;
}

void Environment::updateProbabilities(const std::vector<float> &probabilities) {
  if(probabilities.size()!=this->probabilities.size()) {
    std::cerr << "ERROR: parameter do not matches the current dimension of environment probabilities." << std::endl;
    return;
  }
  if(std::any_of(probabilities.begin(), probabilities.end(), [](float p) { return std::isnan(p); })) {
    // std::cerr << "ERROR: Environment::updateProbabilities parameter contains invalid elements." << std::endl;
    return;
  }

  if(!isPMF(probabilities)) {
    std::cerr << "Update environment probability failed!" << std::endl;
    return;
  }

  for(uint i = 0; i < this->probabilities.size(); i++) {
    this->probabilities.at(i) = (1. - gConfig.lambda)*this->probabilities.at(i) + gConfig.lambda*probabilities.at(i);
  }
  assert(isPMF(this->probabilities) && "environment probability is not a PMF!");
}

std::vector<float> Environment::getPMF() {
  return calculatePMF(environmentTracking_);
}
