#ifndef INCLUDE_SUMOCONNECTOR_H_
#define INCLUDE_SUMOCONNECTOR_H_

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include "../lib/TraCIAPI.h"
#include <experimental/filesystem>
#include <fstream>
#include <cstring>

#include "Util.h"
#include "ISumo.h"
#include "ISimulationObject.h"

/** @class SUMOConnector
 * @brief Wraps the TraCI client.
 *
 * @details The class implements some performance improvements.
 * We use the TraCI Subscription to reduce get requests.
 * We also implement caching of frequently used SUMO variables.
 */
class SUMOConnector : TraCIAPI, public ISumo, public ISimulationObject {
  int pid_{-1};
  int port_{-1};
  bool useGui{false};

  double timeStep{-1};
  double time{0};

  /// Needed for TraCI Subscription.
  size_t startSubscription{0};
  size_t endSubscription{gConfig.simulationTime*STEP_SIZE};

  std::string config_;
  std::string filename_;
  std::string ip_address_;
  std::ofstream log;

  /// STATS
  size_t totalHaltingNumber{};
  size_t totalVehicleNumber{};
  double totalMeanSpeed{};
  double totalWaitingTime{0.};
  double totalAccWaitingTime{0.};
  double totalWaitingTimePerVehicle{0.};
  double totalAccWaitingTimePerVehicle{0.};
  double costs{};

  size_t totalDeviation{0};

  /// SUBSCRIBE to variables
  const std::vector<int> edgeVars = {libsumo::LAST_STEP_VEHICLE_HALTING_NUMBER,
                                     libsumo::LAST_STEP_VEHICLE_NUMBER,
                                     libsumo::LAST_STEP_MEAN_SPEED,
                                     libsumo::LAST_STEP_VEHICLE_ID_LIST};

  const std::vector<int> &laneVars = edgeVars;

  const std::vector<int> vehicleVars = {libsumo::VAR_WAITING_TIME,
                                        libsumo::VAR_ACCUMULATED_WAITING_TIME,
                                        libsumo::VAR_LANE_ID};

  const std::vector<int> trafficLightVars = {libsumo::TL_CURRENT_PHASE,
                                             libsumo::TL_CURRENT_PROGRAM};

  const std::vector<int> guiVars = {libsumo::VAR_VIEW_ZOOM,
                                    libsumo::VAR_VIEW_OFFSET};

  /// SUBSCRIBED values
  libsumo::SubscriptionResults vehResult;
  libsumo::SubscriptionResults edgeResult;
  libsumo::SubscriptionResults laneResult;
  libsumo::SubscriptionResults tlResult;
  libsumo::SubscriptionResults guiResult;

  /// CACHED
  std::vector<std::string> lanes;
  std::vector<std::string> edges;
  std::vector<std::string> junctions;
  std::vector<std::string> trafficLightIDs;

  std::map<std::string, std::vector<libsumo::TraCIConnection>> laneLinks;
  std::map<std::string, std::set<std::string>> tlsJunctions;

  /// TRACE VEHICLES WITH TRACI SUBSCRIPTION
  std::vector<std::string> departedVehicles{};
  std::vector<std::string> arrivedVehicles{};
  std::set<std::string> vehicles;

  /// GUI SUPPORT
  std::pair<int, int> sumoGuiWindowSize{-1, -1};
  std::pair<int, int> sumoGuiWindowPos{-1, -1};

  /// PRIORITIZE PUBLIC TRANSPORT, filter for vehicle types.
  std::string vehicleFilterLabel = "BUS";
  std::set<std::string> filteredVehicles;

  std::map<std::string, int> filteredVehiclesHaltingNumberPerLane;
  std::map<std::string, int> filteredVehiclesVehicleNumberPerLane;

  size_t filteredVehiclesTotalWaitingTime{};
  size_t filteredVehiclesTotalAccWaitingTime{};

  std::map<std::string, std::string> vehIDtoLaneID;
  std::map<std::string, int> filteredAccVehiclesHaltingNumberPerLane;

  /// VIRTUAL LANE FEATURE, contains normal lane IDs where virtual lanes will be added.
  /// lane IDs in this list get an parallel virtual lane in the shield,
  /// which enables to model additional information.
  std::set<std::string> virtualLanes;

 public:
  /** @brief Constructor for the SUMOConnector Class.
   *
   * @param config A String with SUMO configuration filename.
   * @param logFile A String with simulation log filename.
   * @param port A Integer with the port number.
   * @param gui A Boolean flag which enables the SUMO GUI.
   */
  SUMOConnector(const std::string &config, const std::string &logFile, int port, bool gui);

  /// @brief Start the SUMO process.
  void boot();

  /// @brief Connecto to SUMO and load.
  void connect();

  /// @brief Kill the SUMO process.
  void closeAndExit();

  /// @brief Track after simulation step, update tracked and cached variables.
  void track();

  /// @brief Log a simulation step.
  void logSimulation();

  /// @brief Build a lookup table which maps junction name to tlsID.
  void mapJunctionToTls();

  /// @brief Block the given Lane ID.
  void blockLane(const std::string &laneID);

  /// @brief Init the TraCI subscription
  void setupSubscribe();

  /// @brief DEBUG Method which checks if subscription values match which normal TraCI request.
  void checkSubscriptionResults();

  /// @brief Get the Integer subscription result.
  static int getIntFromTraCIResult(libsumo::TraCIResult *result);

  /// @brief Get the Double subscription result.
  static double getDoubleFromTraCIResult(libsumo::TraCIResult *result);

  /// @brief Get a list of Strings of subscription result.
  static std::vector<std::string> getStringListFromTraCIResult(libsumo::TraCIResult *result);

  // IMPLEMENT ISimulationObject INTERFACE

  /// @brief Simulation Step Method, called in Simulation Class.
  void step() override;

  // IMPLEMENT ISumo INTERFACE

  /// @brief Get the current simulation time step.
  /// NOTE: Due to the sumo subscription the simulation the 0 time step is at sumo time 1.
  double getTimeStep() const override;

  /// @brief Get the current sumo time in s.
  double getTime() const override;

  /// @brief Get the current sumo time in ms.
  int getCurrentTime() const override;

  std::vector<std::string> getLaneIDs() override;
  std::vector<std::string> getEdgeIDs() override;
  std::vector<std::string> getTrafficLightIDs() override;
  std::set<std::string> getVehicleIDs() override;

  int getTrafficLightCurrentPhase(const std::string &tlsID) override;
  std::string getTrafficLightCurrentProgram(const std::string &tlsID) override;
  std::vector<std::string> getTrafficLightsControlledLanes(const std::string &tlsID) const override;
  std::vector<std::vector<libsumo::TraCILink>> getTrafficLightsControlledLinks(const std::string &tlsID) const override;
  std::vector<libsumo::TraCILogic> getTrafficLightsAllProgramLogics(const std::string &tlsID) const override;

  void setTrafficLightPhase(const std::string &tlsID, int phaseID) override;
  void setTrafficLightProgram(const std::string &tlsID, const std::string &programID) override;
  void setTrafficLightPhaseDuration(const std::string &tlsID, double phaseDuration) override;

  double getLaneLength(const std::string &laneID) const override;
  int getLaneLastStepHaltingNumber(const std::string &laneID) override;
  int getLaneLastStepVehicleNumber(const std::string &laneID) override;
  double getLaneLastMeanSpeed(const std::string &laneID) override;
  std::vector<std::string> getLaneLastStepVehicleIDs(const std::string &laneID) override;

  int getEdgeLastStepHaltingNumber(const std::string &edgeID) override;
  int getEdgeLastStepVehicleNumber(const std::string &edgeID) override;
  double getEdgeLastMeanSpeed(const std::string &edgeID) override;

  double getVehicleWaitingTime(const std::string &vehID) override;
  double getVehicleAccWaitingTime(const std::string &vehID) override;
  std::string getVehicleLaneID(const std::string &vehID) override;

  std::set<std::string> getTlsJunctions(const std::string &tlsID) override;
  libsumo::TraCIPosition getTlsPosition(const std::string &tlsID) override;

  double getPerformance() const override;
  size_t getHaltingNumber() const override;
  size_t getVehicleNumber() const override;
  double getMeanSpeed() const override;

  void rerouteVehicle(const std::string &vehicleID) const override;

  void createPoi(const std::string &poiID, double x, double y, const libsumo::TraCIColor &c,
                 const std::string &type = POI_TYPE, int layer = POI_LAYER,
                 const std::string &imgFile = POI_IMAGE,
                 double width = POI_WIDTH, double height = POI_HEIGHT, double angle = POI_ANGLE) const override;

  void setPoiColor(const std::string &poiID, const libsumo::TraCIColor &c) const override;

  double getGuiZoom() override;
  libsumo::TraCIPosition getGuiOffset() override;

  void setGuiZoom(double zoom) override;
  void setGuiOffset(const libsumo::TraCIPosition &offset) override;

  void setSumoGuiWindowSize(int x, int y) override;
  void setSumoGuiWindowPos(int x, int y) override;

  /// Expose this member for the Side-By-Side View.
  double currentGuiZoom{};
  libsumo::TraCIPosition currentGuiOffset{};

  /// @brief Add an normal lane ID and create a parallel lane ID.
  void addVirtualLane(const std::string &laneID) override;

  /// @brief Create a virtual lane ID from a normal lane ID, in an specific format.
  std::string virtualLaneID(const std::string &laneID) const override;

  /// @brief Check of an lane ID is an virtual lane ID.
  bool isVirtualLaneID(const std::string &laneID) override;

  /// @brief Check if a normal lane ID has an virtual lane.
  bool hasVirtualLane(const std::string &laneID) const override;

  /** @brief Find the Incoming lanes of a lane ID.
   *
   * @details SUMO/TraCI do not provide such a functionality.
   * We can only see lanes where we go.
   *
   * @param nextLane A String with the lane ID
   * @return A list of String with incoming lane IDs.
   */
  std::vector<std::string> findIncomingLanes(const std::string &nextLane) override;

  size_t getTotalDeviation() const;
  float getDeviationPercentage() const;

  void incrementTotalDeviation(size_t inc);


};

#endif //INCLUDE_SUMOCONNECTOR_H_