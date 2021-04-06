#ifndef INCLUDE_SIMULATIONOBJECT_H_
#define INCLUDE_SIMULATIONOBJECT_H_

/** @class ISimulationObject
 * Interface of Simulation Component used in Composite Pattern.
 */
class ISimulationObject {
 public:
  /// @brief Simulation Step Method, called in Simulation Class.
  virtual void step() {};
};

#endif //INCLUDE_SIMULATIONOBJECT_H_
