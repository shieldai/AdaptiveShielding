#ifndef INCLUDE_TRAFFICLIGHT_H_
#define INCLUDE_TRAFFICLIGHT_H_

#include <vector>

#include "ISimulationObject.h"
#include "LaneMapper.h"
#include "PhaseMapper.h"
#include "IconManager.hpp"

class Shield;
class SUMOConnector;

/** @class TrafficLight
 *
 */
class TrafficLight : public ISimulationObject {
 private:
  SUMOConnector *sumo_;
  Shield *shield_;

  int lastShieldAction{-1};
  int lastOverwrittenAction{-1};

  /// Bring the links/incoming lanes in a storm friendly format, there ist a merge flag in the insert method.
  LaneMapper laneMapper;
  PhaseMapper phaseMapper;
  IconManager iconManager;

  std::string tlsID;
  std::vector<std::string> controlledLanes;

  std::ofstream log;

  size_t &updateInterval{gConfig.updateInterval};
  size_t &warmUpTime{gConfig.updateInterval};

  // STATS
  std::vector<int> shieldTracking_;
  int interferenceCount_{};
  float interferenceRate_{};

  bool activeShield{false};

 public:
  /** @brief Factory Method, create a TrafficLight instance.
   *
   * @details With tlsID everything get generated from SUMO information.
   */
  static TrafficLight *build(SUMOConnector *sumo, const std::string &tlsID);

  /** @brief Factory Method, create a TrafficLight instance from a Shield config file.
   *
   * @details With configuration file we generate first all modules and than load the config values.
   * By that we check if the information in the config file matches and there is no problem with the orders and names.
   */
  static TrafficLight *buildFromFile(SUMOConnector *sumo, const std::string &shieldConfigFile);

  /**
   * @brief Constructor of TrafficLight Class.
   * Each Traffic Light has a reference to a SUMO Connector instance, Shield and Lane Mapper.
   *
   * @param sumo A reference to a SUMO Connector instance.
   * @param tls_id A String with the tlsID.
   * @param shieldConfigFile A String with the configuration filename.
   */
  TrafficLight(SUMOConnector *sumo, const std::string &tls_id);

  /// @brief Destructor for TrafficLight Class.
  ~TrafficLight();

  /// @brief Get Shield state, if true active, false inactive.
  bool getShieldState() const;

  /// @brief Get tlsID.
  std::string getTrafficLightID();

  /// @brief Get current junction phase (from simulation).
  int getJunctionPhase();

  /// @brief Get next junction phase, determined from phase ID and state space.
  int getNextPhaseID();

  /// @brief Get the Shield instance.
  Shield *getShield();

  /// @brief Simulation Step Method, called in Simulation Class.
  void step() override;

  /// @brief Simulation Step before shielding activities.
  void track();

  /// @brief Simulation Step after shielding activities.
  void track2(int shieldUpdated, int dev, int action);

  /// @brief Write logging variable labels to log file.
  void logHeader();

  /// @brief Track the current shield action
  void trackShield(int shieldAction);

  /// @brief Track the current interference
  float trackInterference(int interference);

  std::string printEnvironmentState();
  std::string logTracker();

};
#endif //INCLUDE_TRAFFICLIGHT_H_