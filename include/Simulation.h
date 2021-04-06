#ifndef INCLUDE_SIMULATION_H_
#define INCLUDE_SIMULATION_H_

#include "SUMOConnector.h"
#include "TrafficIncidentManager.h"
#include "Util.h"

class TrafficLight;

/** @class Simulation
 * Brings all SimulationObjects together (Composite).
 */
class Simulation : public ISimulationObject {
  SUMOConnector sumo;
  TrafficIncidentManager tim;

  const std::string simulationLogFile;
  const std::set<std::string> &ignoreIDs;
  const std::set<std::string> &shieldedIDs;

  std::vector<ISimulationObject *> trafficLight;

  bool client;

 public:
  /** @brief Constructor for the Simulation Class.
   *
   * @param sumoConfigFile A String with SUMO configuration filename.
   * @param simulationLogFile A String with simulation log filename.
   * @param shieldConfigFiles A String with the configuration filename.
   * @param ignoreIDs A list of tlsIDs or junctionIDs which will be ignored/not shielded.
   * @param shieldedIDs A list of tlsIDs or junctionIDs which only this IDs will be shielded.
   * @param blockEvents A list of block events, which blocks a laneD to certain time and enable rerouting.
   * @param shield A Boolean flag which enables the shield on a traffic light.
   * @param gui A Boolean flag which enables the SUMO GUI.
   * @param port A Integer with the port number.
   * @param xSize SUMO GUI Windows x size.
   * @param ySize SUMO GUI Windows y size.
   * @param xPos SUMO GUI Windows x position.
   * @param yPos SUMO GUI Windows y position.
   */
  Simulation(const std::string &sumoConfigFile,
             const std::string &simulationLogFile,
             const std::vector<std::string> &shieldConfigFiles,
             const std::set<std::string> &ignoreIDs,
             const std::set<std::string> &shieldedIDs,
             const std::vector<struct blockEventInfo> &blockEvents,
             bool client = false,
             bool shield = true,
             bool gui = true,
             int port = 1330,
             int xSize = -1,
             int ySize = -1,
             int xPos = -1,
             int yPos = -1);

  /// @brief Destructor for the Simulation Class.
  ~Simulation();

  /// @brief Get the SUMOConnector instance.
  SUMOConnector &getSumoInstance();

  /// @brief Get the log filename.
  char *getLogFile();

  /// @brief Simulation Step Method, called in Simulation Class.
  void step() override;

  /// @brief Call step method in loop.
  void loop();

 private:
  /// @brief Check if tlsID is in the tlsIDs list.
  bool containsTlsID(const std::set<std::string> &listOfIDs, const std::string &tlsID);
};

#endif //INCLUDE_SIMULATION_H_