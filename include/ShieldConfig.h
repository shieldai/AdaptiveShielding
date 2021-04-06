#ifndef INCLUDE_SHIELDCONFIG_H_
#define INCLUDE_SHIELDCONFIG_H_

#include <vector>
#include <string>

#include "../lib/json.hpp"
using json = nlohmann::json;

class Environment;
class Controller;

/** @class ShieldConfig
 * Handles configuration file.
 */
class ShieldConfig {
  json config;
  std::string configFilename;

 public:
  ShieldConfig() = default;

  /** @brief Constructor for the ShieldConfig Class.
   * Loads the values from file if everything is ok.
   *
   * @param filename A String with shield configuration file name.
   */
  explicit ShieldConfig(const std::string &filename);

  /** @brief Set filename.
  *
  * @param filename A String with shield configuration file name.
  */
  void setFilename(const std::string &filename);

  /** @brief Get filename.
   *
   * @return A String with shield configuration file name.
   */
  const std::string &getFilename() const;

  /// @brief Write all Shield modules and settings to the JSON file.
  virtual void writeJson();
  /// @brief Read the JSON file and parse all modules.
  virtual void readJson();

  /** @brief Get active flag from JSON file, determines if the shiled will be used.
   *
   * @return Boolean flag.
   */
  bool parseState();

  /** @brief Get modelType string from JSON file, defines the STORM model.
   *
   * @return String with junction name.
   */
  std::string parseModelType();

  /** @brief Get setting string from JSON file, containing the junction name.
   *
   * @return String with junction name.
   */
  std::string parseSetting();

  /** @brief Get properties line from JSON file.
   *
   * @return vector with properties line string.
   */
  std::vector<std::string> parseProperties();

  /** @brief Get arbiter lines from JSON file.
   *
   * @return vector with arbiter line string.
   */
  std::vector<std::string> parseModulesArbiter();

  /** @brief Get environment lines from JSON file.
   *
   * @return vector with environment line string.
   */
  std::vector<std::string> parseModulesEnvironment();

  /** @brief Get controller lines from JSON file.
   *
   * @return vector with controller line string.
   */
  std::vector<std::string> parseModulesController();

  /** @brief Get shield lines from JSON file.
   *
   * @return vector with shield line string.
   */
  std::vector<std::string> parseModulesShield();

  /** @brief Get reward lines from JSON file.
   *
   * @return vector with reward line string.
   */
  std::vector<std::string> parseModulesRewards();

  /** @brief Create Controller Object from JSON config.
   *
   * @return Controller instance.
   */
  Controller parseController();

  /** @brief Create Environment Object from JSON config.
   *
   * @return Environment instance.
   */
  Environment parseEnvironment();

 protected:
  /// Setter in same manner as Getter ...
  void storeModelType(std::string modelType);
  void storeSetting(std::string tlsID, bool state);
  void storeProperties(std::vector<std::string> module);
  void storeModulesArbiter(std::vector<std::string> module);
  void storeModulesEnvironment(std::vector<std::string> module);
  void storeModulesController(std::vector<std::string> module);
  void storeModulesShield(std::vector<std::string> module);
  void storeModulesRewards(std::vector<std::string> module);
  void storeController(Controller &controller);
  void storeEnvironment(Environment &environment);
};

#endif //INCLUDE_SHIELDCONFIG_H_

