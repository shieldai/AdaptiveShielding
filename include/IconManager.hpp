#ifndef INCLUDE_ICONMANAGER_H_
#define INCLUDE_ICONMANAGER_H_

#include <vector>
#include <string>
#include <map>

#include "SUMOConnector.h"

/** @class IconManager
 * A wrapper to create and change SUMO POI Objects.
 */
class IconManager : ISimulationObject {
  SUMOConnector *sumo;
  std::string poiID;
  size_t showIconCountdown{};

 public:

  /** @brief Constructor for the IconManager Class.
   *
   * @param sumo  Pointer to a SUMOConnector instance.
   * @param tls_id  String with the current SUMO Traffic Light ID.
   */
  IconManager(SUMOConnector *sumo, const std::string &tls_id) :
      sumo(sumo), poiID("poi" + tls_id) {
    try {
      auto pos = sumo->getTlsPosition(tls_id);
      sumo->createPoi(poiID, pos.x, pos.y, POI_COLOR_TRANSPARENT);
    }
    catch(std::exception &e) {
      std::cerr << "No junction id found for traffic light id " << tls_id << std::endl;
      poiID.clear();
    }
  }

  /// Simulation step method.
  void step() override {
    if(!poiID.empty()) {
      if(showIconCountdown==0) {
        hideIcon();
      } else {
        showIconCountdown--;
      }
    }
  }

  /// @brief set the POI visible and set up the countdown.
  void showIcon() {
    if(!poiID.empty()) {
      sumo->setPoiColor(poiID, POI_COLOR_UPDATE);
      showIconCountdown = STEP_IN_DELTA;
    }
  }

  /// @brief set the POI transparent.
  void hideIcon() {
    if(!poiID.empty()) {
      sumo->setPoiColor(poiID, POI_COLOR_TRANSPARENT);
    }
  }
};

#endif //INCLUDE_ICONMANAGER_H_
