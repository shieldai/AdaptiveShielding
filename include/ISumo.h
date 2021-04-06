#ifndef INCLUDE_TRACIFLATINTERFACE_H_
#define INCLUDE_TRACIFLATINTERFACE_H_

#include <set>
#include <string>
#include "../lib/TraCIAPI.h"

/** @class ISumo
 * Wrap up TraCI with a easy and flat structure.
 * This enables to create a simple adapter and furthermore caching and lazy acquire.
 */
class ISumo {
 public:
  virtual double getTimeStep() const = 0;
  virtual double getTime() const = 0;
  virtual int getCurrentTime() const = 0;

  virtual std::vector<std::string> getLaneIDs() = 0;
  virtual std::vector<std::string> getEdgeIDs() = 0;
  virtual std::vector<std::string> getTrafficLightIDs() = 0;
  virtual std::set<std::string> getVehicleIDs() = 0;

  virtual int getTrafficLightCurrentPhase(const std::string &tlsID) = 0;
  virtual std::string getTrafficLightCurrentProgram(const std::string &tlsID) = 0;
  virtual std::vector<std::string> getTrafficLightsControlledLanes(const std::string &tlsID) const = 0;
  virtual std::vector<std::vector<libsumo::TraCILink>> getTrafficLightsControlledLinks(const std::string &tlsID) const = 0;
  virtual std::vector<libsumo::TraCILogic> getTrafficLightsAllProgramLogics(const std::string &tlsID) const = 0;

  virtual void setTrafficLightPhase(const std::string &tlsID, int phaseID) = 0;
  virtual void setTrafficLightProgram(const std::string &tlsID, const std::string &programID) = 0;
  virtual void setTrafficLightPhaseDuration(const std::string &tlsID, double phaseDuration) = 0;

  virtual double getLaneLength(const std::string &laneID) const = 0;
  virtual int getLaneLastStepHaltingNumber(const std::string &laneID) = 0;
  virtual int getLaneLastStepVehicleNumber(const std::string &laneID) = 0;
  virtual double getLaneLastMeanSpeed(const std::string &laneID) = 0;
  virtual std::vector<std::string> getLaneLastStepVehicleIDs(const std::string &laneID) = 0;

  virtual int getEdgeLastStepHaltingNumber(const std::string &edgeID) = 0;
  virtual int getEdgeLastStepVehicleNumber(const std::string &edgeID) = 0;
  virtual double getEdgeLastMeanSpeed(const std::string &edgeID) = 0;

  virtual double getVehicleWaitingTime(const std::string &vehID) = 0;
  virtual double getVehicleAccWaitingTime(const std::string &vehID) = 0;
  virtual std::string getVehicleLaneID(const std::string &vehID) = 0;

  virtual std::set<std::string> getTlsJunctions(const std::string &tlsID) = 0;
  virtual libsumo::TraCIPosition getTlsPosition(const std::string &tlsID) = 0;

  virtual double getPerformance() const = 0;
  virtual size_t getHaltingNumber() const = 0;
  virtual size_t getVehicleNumber() const = 0;
  virtual double getMeanSpeed() const = 0;

  virtual void rerouteVehicle(const std::string &vehicleID) const = 0;

  virtual void createPoi(const std::string &poiID,
                         double x,
                         double y,
                         const libsumo::TraCIColor &c,
                         const std::string &type,
                         int layer,
                         const std::string &imgFile,
                         double width,
                         double height,
                         double angle) const = 0;

  virtual void setPoiColor(const std::string &poiID, const libsumo::TraCIColor &c) const = 0;

  virtual double getGuiZoom() = 0;
  virtual libsumo::TraCIPosition getGuiOffset() = 0;

  virtual void setGuiZoom(double zoom) = 0;
  virtual void setGuiOffset(const libsumo::TraCIPosition &offset) = 0;

  virtual void setSumoGuiWindowSize(int x, int y) = 0;
  virtual void setSumoGuiWindowPos(int x, int y) = 0;

  virtual void addVirtualLane(const std::string &laneID) = 0;
  virtual std::string virtualLaneID(const std::string &laneID) const = 0;
  virtual bool isVirtualLaneID(const std::string &laneID) = 0;
  virtual bool hasVirtualLane(const std::string &laneID) const = 0;

  virtual std::vector<std::string> findIncomingLanes(const std::string &nextLane) = 0;
};

#endif //INCLUDE_TRACIFLATINTERFACE_H_
