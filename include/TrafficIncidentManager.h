#ifndef INCLUDE_DYNAMICREROUTE_H_
#define INCLUDE_DYNAMICREROUTE_H_

#include <vector>
#include <algorithm>
#include <string>

#include "SUMOConnector.h"
#include "Util.h"
#include "ISimulationObject.h"

/** @class DynamicReroute
 * Reroutes subscribed lanes, leaking functionality in the TraCI API.
 */
class DynamicReroute : public ISimulationObject {
  /// Keep track of lanes where we have to reroute.
  std::vector<std::string> laneIDs;
  /// Cache already rerouted vehicles.
  std::vector<std::string> reroutedIDs;

 protected:
  SUMOConnector &sumo;

 public:
  /** @brief Constructor for the DynamicReroute Class.
   *
   * @param sumo A SUMO Connector instance.
   */
  explicit DynamicReroute(SUMOConnector &sumo);

  /** @brief Add a lane ID to the subscription list.
   *
   * @param laneID A String with the lane ID.
   */
  void addRerouting(const std::string &laneID);

  /// @brief Simulation Step Method, do rerouting for lanes in the subscription list.
  void step() override;
};

/** @class TrafficIncidentManager
 * Manage Traffic Incidents, currently only blocking roads.
 */
class TrafficIncidentManager : DynamicReroute {
  const std::vector<struct blockEventInfo> &blockEvents;

 public:
  /** @brief Constructor for the DynamicReroute Class.
   *
   * @param sumo A SUMO Connector instance.
   * @param blockEvents A list of blocking events.
   */
  TrafficIncidentManager(SUMOConnector &sumo, const std::vector<struct blockEventInfo> &blockEvents);

  /// @brief Simulation Step Method, check blocking events and do rerouting.
  void step() override;
};

#endif //INCLUDE_DYNAMICREROUTE_H_
