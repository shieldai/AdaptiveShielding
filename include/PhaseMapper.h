#ifndef INCLUDE_PHASEMAPPER_H_
#define INCLUDE_PHASEMAPPER_H_

#include <vector>
#include <string>
#include <map>
#include "ISimulationObject.h"

class ISumo;

/** @class PhaseMapper
 *
 */
class PhaseMapper : ISimulationObject {
  ISumo *sumo{};
  std::string tlsID;

  std::vector<struct phaseInfo> phases;

  /// @brief Map SUMO phase ID to internal phase ID.
  std::map<int, int> sumoPhaseIDtoPhaseID;
  /// @brief Map internal phase ID (= vector index) to SUMO phase ID (= vector value).
  std::vector<int> sumoPhaseID;

  /// @brief Map SUMO phase ID to the phase duration.
  std::map<int, double> sumoPhaseIDtoDuration;
  /// @brief Map internal phase ID to the phase duration.
  std::map<int, double> phaseIDtoDuration;

  /// @brief Internal STATIC Controller (shadows real) to correctly restore.
  int controllerPhase{0};
  int controllerSumoPhase{0};
  int phaseDurationCountdown{0};
  int sumoPhaseDurationCountdown{0};

 public:
  PhaseMapper() = default;

  /** @brief Constructor for the PhaseMapper Class.
   *
   * @param sumo  Pointer to a SUMOConnector instance.
   * @param tls_id  String with the current SUMO Traffic Light ID.
   * @param links A list of lists with Strings which represents the links, similar to SUMO links but formatted.
   */
  PhaseMapper(ISumo *sumo, const std::string &tls_id, const std::vector<std::vector<std::string>> &links);

  /// @brief Step method, should be called in each tie step, updates the internal static controller.
  void step() override;

  /** @brief Get all lane labels of junction which will be controlled.
   *
   * @param links A list of lists with Strings which represents the links, similar to SUMO links but formatted.
   * @param state A string with the SUMO state string.
   * @return A list with String off all used lanes.
   */
  std::vector<std::string> getActiveLaneLabels(const std::vector<std::vector<std::string>> &links,
                                               const std::string &state);

  /** @brief Get controller phases as vector of phase info structs.
   *
   * @return A list of phase info structs.
   */
  const std::vector<struct phaseInfo> &getPhases();

  /** @ Get the internally shadowed controller value.
   *
   * @return A Integer with the shadowed internal controller value.
   */
  int getControllerPhase() const;

  /** @brief Get the current controller phase ID in SUMO format.
   *
   * @return SUMO phase index.
   */
  int getSumoPhaseID(int phaseID) const;

  /** @brief Get the current controller phase ID in internal format.
   *
   * @return INTERNAL phase index.
   */
  int getPhaseID() const;

  /** @brief Set the current controller phase ID in internal format.
   *
   * @param phaseID A Integer with the next internal phase ID.
   */
  void setPhaseID(int phaseID) const;

  /// @brief Reset controller with internally shadowed values.
  void restoreController();

  /// @brief Reset controller program to default.
  void resetProgram();
};

#endif //INCLUDE_PHASEMAPPER_H_
