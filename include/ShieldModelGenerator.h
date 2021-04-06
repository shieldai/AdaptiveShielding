#ifndef INCLUDE_SHIELDMODELFILEGENERATOR_H_
#define INCLUDE_SHIELDMODELFILEGENERATOR_H_

#include <vector>
#include <string>

class Environment;
class Controller;

/** @class ShieldModelGenerator
 * Handles PRISM file generation.
 */
class ShieldModelGenerator {
  std::string filenamePrefix;
 protected:
  /// PRISM PROPS
  std::string modelType_;
  std::vector<std::string> properties;

  /// PRISM SYNTAX
  std::vector<std::string> module_arbiter;
  std::vector<std::string> module_environment;
  std::vector<std::string> module_controller;
  std::vector<std::string> module_shield;
  std::vector<std::string> module_rewards;

 public:
  ShieldModelGenerator() = default;

  /** @brief Constructor for the ShieldModelGenerator Class.
   *
   * @param filenamePrefix A String with file prefix, usually the tlsID.
   * Derives the .prism and .props file from it.
   */
  explicit ShieldModelGenerator(const std::string &filenamePrefix);

  /// @brief Create PROPS file for STORM.
  void createPropFile();

  /// @brief Create PRISM file for STORM.
  void createPRISMFile(const Environment &environment, const Controller &controller);

  /// @brief Create arbiter string for PRISM File.
  void createPRISMArbiter(const Controller &controller);

  /// @brief Create environment string for PRISM File.
  void createPRISMEnvironment(const Environment &environment);

  /// @brief Create controller string for PRISM File.
  void createPRISMController(const Controller &controller);

  /// @brief Create shield string for PRISM File.
  void createPRISMShield(const Controller &controller);

  /// Create reward string for PRISM File.
  void createPRISMRewards(const Controller &controller);

  /// METHODS ABOVE Generate Model from the environment.
  /// Load Model from config file and set it. NO FUTURE SYNTAX CHECKS
  void setModelType(std::string modelType);
  void setProperties(std::vector<std::string> module);
  void setPRISMArbiter(const std::vector<std::string> &module);
  void setPRISMEnvironment(const std::vector<std::string> &module);
  void setPRISMController(const std::vector<std::string> &module);
  void setPRISMShield(const std::vector<std::string> &module);
  void setPRISMRewards(const std::vector<std::string> &module);
};

#endif //INCLUDE_SHIELDMODELFILEGENERATOR_H_
