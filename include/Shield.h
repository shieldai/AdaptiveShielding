#ifndef INCLUDE_SHIELD_H_
#define INCLUDE_SHIELD_H_

#include <vector>
#include <string>

#include "Environment.h"
#include "Controller.h"
#include "Strategy.h"

#include "ShieldConfig.h"
#include "ShieldModelGenerator.h"

/** @class Shield
 * Keeps track of Controller and Environment and holds the Shield Strategy.
 * Creates the PRISM Files for the STORM Model Checker.
 * Creates and loads the Shield configuration file.
 */
class Shield : public ShieldConfig, ShieldModelGenerator {
  std::string tlsID;
  bool active;

  Controller controller;
  Environment environment;
  Strategy strategy;

  /// Keep track of states in case of roll back
  /// Implement momento pattern in multithreading applications.
  std::vector<float> lastEnvironmentProbabilities;
  std::vector<int> lastStateSpace;
  std::vector<int> newStateSpace;
  bool maxStateSpaceSizeReached{false};

  /// Log the state space history to adapt state space values.
  std::vector<std::vector<int>> stateSpaceHistory;

  /// Log some properties.
  int generation{-1};
  int stateDelta{0};
  float probDelta{0.};

 public:
  /** @brief Constructor for the Shield Class.
   * Initializes the Class with default values (shield not active and no strategy).
   * The remaining fields initialized with default values.
   */
  Shield();

  /** @brief Constructor for the Shield Class.
   * The Controller and Environment instance should be generated from SUMO informations.
   *
   * @param tlsID  String with the current SUMO Traffic Light ID.
   * @param environment A environment object.
   * @param controller A controller object.
   */
  Shield(const std::string &junction, const Environment &environment, const Controller &controller);

  /** @brief Get Shield state.
   *
   * @return A Boolean, True if active, False otherwise.
   */
  bool state() const;

  /// @brief Disable the Shield. Do not allow to reactivate a Shield.
  void disable();

  /** @brief Get the tlsID/junction name.
   *
   * @return A String with tlsID.
   */
  std::string getJunction() const;

  /// @brief Set Environment.
  void setEnvironment(Environment &newEnvironment);

  /// @brief Get a reference of the current Environment instance.
  const Environment *getEnvironment() const;

  /// @brief Set Controller.
  void setController(Controller &newController);

  /// @brief Get a reference of the current Controller instance.
  const Controller *getController() const;

  /// @brief Get the summed change of the state probabilities.
  float getStateProbabilitiesDelta() const;

  /// @brief Get the current shield generation.
  int getShieldGeneration() const;

  /// @brief Print shield properties of Traffic Light.
  void printConfig();

  /// @brief Get a log string of shield properties of Traffic Light.
  /// Similar to printConfig but this function fixes precision
  /// which can cause troubles with test script
  std::string logConfig();

  /** @brief Creates a new Strategy with STORM Model Checker and overwrites the old.
   *
   * @details This method wraps the Singleton function call,
   * which triggers the fork of STORM with the generated PRISM files.
   * After STORM finished the Strategy gets updated.
   * BLOCKING until STORM process finished!
   */
  void createStrategy();

  /** @brief Get the reference of the current Strategy instance.
   *
   * @return A reference to the Strategy instance.
   */
  Strategy *getStrategy();

  /** @brief Updates the reference to the Strategy instance.
   * This method will be called by the STORMConnector.
   */
  void updateStrategyCallback();

  /** @brief Lock the state space size and avoid future growth.
   * The method wil be called when STORM timeouts or fails due to the complexity of the model.
   */
  void lockStateSpaceSize();

  /** @brief Define needed state space from observations.
   *
   * @details The state space can grow and this method find a new state space size
   * on observation +1, which defines satisfies future limits.
   *
   * @return A list of Integer with state space size.
   */
  std::vector<int> checkStateInfo();

  /// @brief Update the state space size on observations.
  void updateStateSpace();

  /// @brief Update the state space probabilities on observations.
  void updateStateProbabilities();

  /// @brief Update the action space probabilities on observations.
  void updateActionProbabilities();

  /// @brief Update the halting vehicles number.
  void updateHaltingNumbers(const std::vector<int> &haltingNumbers);

  /// @brief Update the vehicles number.
  void updateVehicleNumbers(const std::vector<int> &vehicleNumbers);

  /// @brief Update the junction phase.
  void updateJunctionPhase(int currentJunctionPhase);

  /// @brief Update Shield properties on observations.
  void update();

  /** @brief Get the next action from the strategy, determined from current state space (= halting vehicle number)
   * and current traffic light controller phase.
   *
   * @param currentAction  Current traffic light controller phase.
   * @return new controller phase determined by the strategy.
   */
  int getNextAction(int currentAction);

  /// Adapt config file reading.
  /// @brief Read the JSON file and parse all modules.
  void readJson() override;

  /// Adapt config file writing.
  /// @brief Write all modules properties to the JSON file.
  void writeJson() override;
};

/** @brief Factory Method to create a shield from config file.
 *
 * @param shieldConfigFile A String with the configuration filename.
 * @return A new Shield instance
 */
[[deprecated("NOT MAINTAINED.")]]
Shield *buildFromFile(const std::string &shieldConfigFile);

#endif //INCLUDE_SHIELD_H_