#ifndef INCLUDE_UTIL_H_
#define INCLUDE_UTIL_H_

#include <vector>
#include <string>
#include <map>
#include <set>

#define out_path_ "out/"

#define EPSILON 1.0e-5

#define DEFAULT_MAX_STEPS 50000
#define STEP_SIZE 1000

// FIX STATE SPACE by DEFAULT_MAX_LANE_SIZE == MIN_LANE_SIZE
// the max lane size can also make problems in osm
// since we build tres of previous lines we can get here errors
#define DEFAULT_MAX_LANE_SIZE 8
#define MIN_LANE_SIZE 3

#define ALLOW_FAIL_ON_STATE_SPACE_SIZE 1
#define RESET_JSON_DIST 0

#define PRIORITIZE_PUBLIC_TRANSPORT 0

#define MIN_LANE_WEIGHT 1
#define MAX_LANE_WEIGHT 30

#define UPDATE_PROBABILITY_DELTA 0.01
#define UPDATE_STATE_DELTA 1
#define DEFAULT_UPDATE_INTERVAL 500
#define DEFAULT_WARMUP_TIME 900

#define STEP_IN_DELTA 5
#define DEFAULT_LAMBDA 0.2
#define DEFAULT_D 3

// avg length
#define DEFAULT_VEHICLE_LENGTH 5.

#define POI_TYPE "shield"
#define POI_LAYER 2
#define POI_IMAGE "data/shield.bmp"
#define POI_WIDTH 50.0
#define POI_HEIGHT 50.0
#define POI_ANGLE 0.0
#define POI_COLOR_UPDATE libsumo::TraCIColor(153,204,255,180)
#define POI_COLOR_TRANSPARENT libsumo::TraCIColor(153,204,255,0)

/** @struct configInfo
 * Global struct to keep parameters.
 */
struct configInfo {
  std::string sumoConfigFile;
  std::vector<std::string> shieldConfigFiles;

  std::set<std::string> ignoreIDs; // tlsID or junctionID
  std::set<std::string> shieldedIDs; // tlsID or junctionID
  std::vector<struct blockEventInfo> blockEvents;

  std::string logFile;

  size_t updateInterval{DEFAULT_UPDATE_INTERVAL};
  size_t warmUpTime{DEFAULT_WARMUP_TIME};
  int d{DEFAULT_D};
  double lambda{DEFAULT_LAMBDA};
  size_t maxLaneSize{DEFAULT_MAX_LANE_SIZE};
  size_t simulationTime{DEFAULT_MAX_STEPS};
  bool prioritizeBus{PRIORITIZE_PUBLIC_TRANSPORT};
  bool noMerging{false};
  bool unshielded{false};
  bool sideBySide{false};
  bool staticUpdate{false};
  bool gui{false};
  bool noTrees{false};
  bool client{false};
  bool overwrite{false};
  int port{-1};
};

extern struct configInfo gConfig;

/** @struct phaseInfo
 * Helper to hand over traffic light phase information with a fixed order.
 */
struct phaseInfo {
  int phaseID;
  int sumoPhaseID;
  std::string state; // sumo state string
  std::vector<std::string> activeInLanes;
};

/** @struct blockEventInfo
 * Helper to hand over lane blocking information.
 */
struct blockEventInfo {
  std::string blockLaneID{};
  size_t timeStep{};
  std::string rerouteLaneID{};
};

/** @brief Read the IGNORE FILE given by the program argument.
 *
 * @param ignoreFiles A list of Strings with filenames.
 * @return A set of String with junctionIDs and/or tlsIDs.
 */
std::set<std::string> readIgnoreFiles(std::vector<std::string> &ignoreFiles);

/** @brief Read the BLOCKING FILE given by the program argument.
 *
 * @param blockFile A list of Strings with filenames.
 * @return A list of blocking events with tlsID and time.
 */
std::vector<struct blockEventInfo> readBlockFile(std::string &blockFile);

/// @brief Parse the arguments to the global struct.
void parse_args(int argc, char *argv[], struct configInfo &config);

/// @brief Start Python with the plot script with the log file of all simulations.
void pythonPlot(const std::vector<char *> &logFiles);

/// @brief Check if two float values are nearly equal.
bool isNear(float value, float reference);

/// @brief Check if input is probability mass function, sum if inputs is 1.
bool isPMF(std::vector<float> probabilities);

/// @brief Get String between two Strings.
std::string getSubstrBetweenDelims(const std::string &str,
                                   const std::string &start_delim,
                                   const std::string &stop_delim);

/// @brief Get Value from token.
std::string getValueForToken(std::string const &str, std::string const &token);

/// @brief Create a probability mass function from the input.
std::vector<float> calculatePMF(std::vector<int> &tracking);

/// @brief Check if file exists.
bool fileExist(const std::string &path);

/// @brief Get a list of Strings separated by delimiter.
std::vector<std::string> split(const std::string &line, const std::string &delim);

/// @brief Replace in String from old to a new.
void replace(std::string &str, const std::string &from, const std::string &to);

/// @brief Format String in a STORM friendly format.
void formatName(std::string &name);

/// @brief Reformat String from a STORM friendly format back to the SUMO format.
void reformatName(std::string &name);

/// @brief Get a String with the timestamp.
std::string getTimeString();

#endif //INCLUDE_UTIL_H_