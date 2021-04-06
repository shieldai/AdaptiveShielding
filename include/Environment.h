#ifndef INCLUDE_ENVIRONMENT_H_
#define INCLUDE_ENVIRONMENT_H_

#include <vector>
#include <string>
#include <cassert>
#include "Util.h"

/** @class Environment
 * Keeps track of the simulations environment and adapts properties on observations.
 */
class Environment {
 private:
  std::vector<std::string> labels;
  std::vector<float> probabilities;
  std::vector<int> weights;
  std::vector<int> stateSpace;

  // STAT
  std::vector<int> vehicleNumbers;
  std::vector<size_t> allVehicleNumbers;
  std::vector<size_t> allNewVehicleNumbers;
  std::vector<int> haltingNumbers;
  std::vector<int> environmentTracking_;

 public:
  Environment() = default;;

  /** @brief Constructor for the Environment Class.
   * Initializes the Class with given values (labels, probabilities, state space).
   * The remaining fields initialized with default values.
   *
   * @param labels A list of Strings of the lane labels.
   * @param probabilities A list of Floats of the probabilities values.
   * @param weights A list of Integers of the weight values. (UNUSED)
   * @param state_space A list of Integers of the max. state values.
   */
  Environment(const std::vector<std::string> &labels,
              const std::vector<float> &probabilities,
              const std::vector<int> &weights,
              const std::vector<int> &state_space);

  /** @brief Constructor for the Environment Class.
   * Initializes the Class with minimal information (labels).
   * The remaining fields initialized with default values.
   *
   * @param labels A list of Strings of the lane labels.
   * @param weights A list of Integers of the weight values. (UNUSED)
   */
  Environment(std::vector<std::string> &lanes, std::vector<int> &weights);

  /** @brief Check the Class fields.
   *
   * @return True if class is ok, False otherwise.
   */
  bool check() const;

  /** @brief Update weights, dimension is fixed.
   *
   * @param weights A list of Integers with weight values.
   */
  void updateWeights(const std::vector<int> &weights);

  /** @brief Update probabilities, on observation.
   */
  void updateProbabilities();

  /** @brief Update state space, dimension is fixed.
   * The state space should only grow.
   *
   * @param stateSpace A list of Integers of the lane sizes.
   */
  void updateStateSpace(const std::vector<int> &stateSpace);

  /** @brief Set state space.
   *
   * @param stateSpace A list of Integers of the lane sizes.
   */
  void setStateSpace(const std::vector<int> &stateSpace);

  /** @brief Update halting vehicle numbers on lane on observations.
   *
   * @param haltingNumbers A list of Integers with halting vehicle number.
   */
  void updateHaltingNumbers(const std::vector<int> &haltingNumbers);

  /** @brief Update vehicle numbers on lane on observations.
   *
   * @param vehicleNumbers A list of Integers with vehicle number.
   */
  void updateVehicleNumbers(const std::vector<int> &vehicleNumbers);

  /** @brief Used to load values from configuration. Update fields with given Object.
   * The method corrects the probabilities, weights and state space order by identify the labels.
   *
   * @param environment A environment object.
   */
  void updateProperties(Environment &environment);

  /** @brief Get the state space labels.
   *
   * @return A list of Strings of the state space labels.
   */
  std::vector<std::string> getStateSpaceLabels() const;

  /** @brief Get the state space size.
   *
   * @return A list of Integers with the state space size.
   */
  std::vector<int> getStateSpace() const;

  /** @brief Get the weights.
   *
   * @return A list of Integers with weights values.
   */
  std::vector<int> getWeights() const;

  /** @brief Get the probabilities.
   *
   * @return A list of Floats of the probabilities values.
   */
  std::vector<float> getProbabilities() const;

  /** @brief Get the halting vehicles.
   *
   * @return A list of Integers with halting vehicles.
   */
  std::vector<int> getHaltingNumbers();

  /** @brief Get the vehicles.
   *
   * @return A list of Integers with vehicles.
   */
  std::vector<int> getVehicleNumbers();

  /** @brief Get the state space with the probability values.
   *
   * @return A output String.
   */
  std::string getStateSpaceString() const;

  /** @brief Get the state space with the probability values.
   *
   * @return A output String.
   */
  std::string getStateSpaceSizeString() const;

 private:
  /** @brief Update probabilities, dimension is fixed.
   *
   * @param probabilities A list of Floats of the probabilities values.
   */
  void updateProbabilities(const std::vector<float> &probabilities);

  /** @brief Get a normal distributed form the latest lane states.
   *
   * @return A list of Floats of the probabilities values.
   */
  std::vector<float> getPMF();
};

#endif //INCLUDE_ENVIRONMENT_H_
