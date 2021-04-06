#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <vector>
#include <string>
#include "Util.h"

/** @class Controller
 * Keeps track of the simulations controller and adapts properties on observations.
 *
 * In the current version, the controller properties update is not used
 * since we have to deal only with static controllers.
 */
class Controller {
 private:

  /// Action Space - used in SUMO
  std::vector<std::string> actions;
  /// Action Space Labels (index == actionID) - used in STORM
  std::vector<std::string> actionLabels;
  std::vector<float> probabilities;
  std::vector<std::vector<std::string>> ways;

  // STAT
  std::vector<int> junctionPhaseStats; // Histogram of enable phase ids
  int currentJunctionPhase{-1};

 public:
  Controller() = default;;

  /** @brief Constructor for the Controller Class.
   * Initializes the Class with given values (actions, probabilities, ways).
   * The remaining fields initialized with default values.
   *
   * @param actions A list of Strings of the action space labels.
   * @param probabilities A list of Floats of the probabilities values.
   * @param ways A list of lists of Strings of the lane labels.
   */
  Controller(const std::vector<std::string> &actions,
             const std::vector<float> &probabilities,
             const std::vector<std::vector<std::string>> &ways);

  /** @brief Constructor for the Controller Class.
   * Initializes the Class with phase information.
   *
   * @param phases A list phase information structs.
   */
  Controller(const std::vector<struct phaseInfo> &phases);

  /** @brief Check the Class fields.
   *
   * @return True if class is ok, False otherwise.
   */
  bool check() const;

  /** @brief Update probabilities, on observation.
   */
  void updateProbabilities();

  /** @brief Update action space labels, dimension is fixed.
   *
   * @param A list of Strings of the action space labels.
   */
  void updateActionSpace(const std::vector<std::string> &actionSpace);

  /** @brief Update controller junction phase.
   *
   * @param currentJunctionPhase New junction phase value.
   */
  void updateJunctionPhase(int currentJunctionPhase);

  /** @brief Used to load values from configuration. Update fields with given Object.
   * The method corrects the probabilities order and action labels by identify the ways.
   *
   * @param controller A controller object.
   */
  void updateProperties(Controller &controller);

  /** @brief Get the action space labels.
   *
   * @return A list of Strings of the action space labels.
   */
  std::vector<std::string> getActionSpace() const;

  /** @brief Get the action labels.
   *
   * @return A list of Strings of the action labels.
   */
  std::vector<std::string> getActionSpaceLabels() const;

  /** @brief Get the probabilities.
   *
   * @return A list of Floats of the probabilities values.
   */
  std::vector<float> getProbabilities() const;

  /** @brief Get the lane ways of the controller.
   *
   * @return A list of lists of Strings of the lane labels.
   */
  std::vector<std::vector<std::string>> getActionSpaceWays() const;

  /** @brief Get the current controller phase ID.
   *
   * @return INTERNAL phase index.
   */
  int getJunctionPhase() const;

  /** @brief Get the action space with the probability values.
   *
   * @return A output String.
   */
  std::string getActionSpaceString();

 private:
  /** @brief Update probabilities, dimension is fixed.
   *
   * @param probabilities A list of Floats of the probabilities values.
   */
  void updateProbabilities(const std::vector<float> &probabilities);

  /** @brief Get a normal distributed form the latest phase states.
   *
   * @return A list of Floats of the probabilities values.
   */
  std::vector<float> getPMF();
};

#endif //INCLUDE_CONTROLLER_H_
