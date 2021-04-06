#ifndef INCLUDE_LANEMAPPER_H_
#define INCLUDE_LANEMAPPER_H_

#include <vector>
#include <string>
#include <set>
#include <map>

#include "Util.h"
#include "LaneTree.h"

// Forward declaration
namespace libsumo {
class TraCILogic;
}

class ISumo;

/**
 * Struct laneInfo. Contains the SUMO lane label and LaneTree for an internal lane label.
 */
struct laneInfo {
  std::set<std::string> sumoLabels;
  std::set<LaneTree *> sumoLabelsTree;
};

/** @class LaneMapper
 * Creates a mapping of SUMO lane labels to internal lane label.
 *
 * The merging of parallel SUMO lanes is supported and are abstracted by internal lane label.
 * For all SUMO lanes abstracted by the LaneMapper we hold a LaneTree to enable the to get the
 * halting vehicle and vehicle number on lanes behind the first lane connected to the traffic light.
 */
class LaneMapper {
  ISumo *sumo{};
  std::string tlsID;
  std::map<std::string, struct laneInfo> m;
  std::map<std::string, std::set<int>> laneControlledInPhase;
  int non_yellow{};

  std::vector<std::vector<std::string>> formattedIncomingLanesPerPhase;
 public:
  LaneMapper() = default;

  /** @brief Constructor for the LaneMapper Class.
   * Uses the SUMOConnector instance to create the lane mapping of the Traffic Light.
   *
   * @param sumo  Pointer to a SUMOConnector instance.
   * @param tls_id  String with the current SUMO Traffic Light ID.
   */
  LaneMapper(ISumo *sumo, const std::string &tls_id);

  /// @brief Destructor for the LaneMapper Class. Delete the dynamic memory.
  ~LaneMapper();

  /** @brief Get all labels which abstract SUMO labels.
   * The labels are formatted to use them also as STORM variable.
   *
   * @return  A list of Strings of the labels.
   */
  std::vector<std::string> getLabels();

  /** @brief Get all lane labels of a internal lane (more than one of merged).
   *
   * @param lane  String of lane label.
   * @return A Set of laneIDs.
   */
  std::set<std::string> getSumoLabel(const std::string &lane);

  /** @brief Get all LaneTrees of one lane label.
   *
   * @param lane  String of lane label.
   * @return A Set of Pointers of the LaneTree objects.
   */
  std::set<LaneTree *> getSumoLabelTree(const std::string &lane);

  /** @brief Get links from the loaded junction, string formation and merging is done.
   *
   * @return A list of list with formatted lane string
   */
  const std::vector<std::vector<std::string>> &getFormattedLinks();

 private:
  /** @brief Collect information of the Traffic Light Phase.
   * Analyse yellow phase and in which phase the incoming ales are enable.
   *
   * @param tlsID  String with the current SUMO Traffic Light ID.
   */
  void loadTrafficLightPhaseInfo(const std::string &tlsID);

  /** @brief Initialize the lane mapper.
   *
   * @param tlsID  String with the current SUMO Traffic Light ID.
   */
  void initLaneMapper(const std::string &tls_id);

  /// @brief Check all LaneTrees for errors and unsupported scenarios.
  void testLaneTrees();

  /** @brief Insert a SUMO lane label in the LaneMapper.
   * Support the merging of parallel lanes to decrease the state space.
   * Supports to create additional virtual lanes to model more properties.
   *
   * @param sumoLaneLabel
   * @param merge  Boolean flag which allows the merging of parallel lanes.
   * @param virt  Boolean flag marks a lane as virtual a creates an additional entry for an parallel virtual lane.
   * @return A String of the formated label for internal use.
   */
  std::string insert(const std::string &sumoLaneLabel, bool merge = true, bool virt = false);

  /** @brief Check if the traffic light is supported.
   *
   * @param tlsLogic The tls logic of type libsumo::TraCILogic.
   * @return A Boolean, true if everything is ok, false otherwise.
   */
  bool checkTrafficLightProgram(libsumo::TraCILogic tlsLogic);
};

#endif //INCLUDE_LANEMAPPER_H_
