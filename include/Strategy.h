#ifndef INCLUDE_STRATEGY_H_
#define INCLUDE_STRATEGY_H_

#include <map>
#include <utility>
#include <vector>
#include <string>

/** @class Strategy
 * Contains the Shield Strategy of the Model Checker.
 * Loads the .sched file from the model checker with the strategy/shield actions,
 * and writes the Strategy to the .start file.
 */
class Strategy {
 private:
  std::vector<std::string> labels;
  std::string filePrefix;

  /// Map <current state space, current action> to the shield action.
  std::map<std::pair<std::vector<int>, int>, int> strategy_;

 public:
  Strategy() = default;

  /** @brief Constructor for the Strategy Class.
   * Loads the strategy from the .sched file and writes it to the .strat file.
   *
   * @param filePrefix A String with filename prefix, usually the tlsID.
   * @param labels A list of String with the lane labels.
   */
  Strategy(const std::string &filePrefix, const std::vector<std::string> &labels);

  /** @brief Check the Class fields.
   *
   * @return True if class is ok, False otherwise.
   */
  bool check() const;

  /** @brief Add a Strategy step to the Strategy.
   *
   * @param state A list of Integer with state values.
   * @param currentAction A Integer with the current action/phase index.
   * @param nextAction A Integer with th next action/phase index.
   */
  void addStrategyStep(const std::vector<int> &state, int currentAction, int nextAction);

  /** @brief Get the Strategy action on the simulation state (state,action).
   *
   * @param simulationState A pair of a state list and action.
   * @return A Integer with the next action according to the Strategy.
   */
  int getStrategyAction(std::pair<std::vector<int>, int> simulationState);

  /** @brief Get the Strategy action on the current state and action of the simulation.
   *
   * @param state A list of Integer with state values.
   * @param currentAction A Integer with the current action/phase index.
   * @return A Integer with the next action according to the Strategy.
   */
  int getStrategyAction(const std::vector<int> &state, int currentAction);

  /// @brief Export the parsed Strategy in a user friendly format.
  void exportStrategy();

  /// @brief Load the .sched file in the Strategy instance.
  void loadSchedFile();

  /** @brief Parse a lane of the .shed file.
   *
   * @param line A String with the line of the .sched file.
   * @param state A list of Integer filled by the method.
   * @param currentAction A Integer with the current action/phase index set by the method.
   * @param nextAction A Integer with th next action/phase index set by the method.
   * @return A Boolean, True if the line is successfully parsed, False otherwise.
   */
  bool parseSchedFileLine(const std::string &line, std::vector<int> &state, int &currentAction, int &nextAction);
};
#endif //INCLUDE_STRATEGY_H_