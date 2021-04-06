#ifndef INCLUDE_STORMCONNECTOR_H_
#define INCLUDE_STORMCONNECTOR_H_

#include <iostream>
#include <map>

class Shield;

/**
 * Struct STORMJob. Contains information of a running STORM process.
 */
struct STORMJob {
  pid_t pid;
  clock_t start;
  //clock_t end;
  //int exit;
};

/** @class STORMConnector
 * Singleton which starts STORM processes and keeps track of them.
 */
class STORMConnector {
 private:
  // Every instance should have max. one active job.
  std::map<Shield *, struct STORMJob> jobs;

 public:
  /// @brief Get the Singleton instance
  static STORMConnector &instance() {
    static STORMConnector _instance;
    return _instance;
  }

  ~STORMConnector() = default;

  /** @brief Start a Strategy Update.
   * This method forks STORM, waits until it is finished and
   * triggers the loading of the new strategy.
   *
   * @param shield A reference to the Shield instance.
   */
  void startStrategyUpdate(Shield *update);

 private:
  /// Hide from user.
  STORMConnector() = default;
  STORMConnector(const STORMConnector &) = delete;
  STORMConnector &operator=(const STORMConnector &) = delete;

  /** @brief Static methods forks STORM with arguments and return the pid.
   *
   * @param filePrefix A String with the file prefix of the PRISM files.
   * @return A pid of the forked STORM process.
   */
  static pid_t startStorm(std::string &filePrefix);

  /** @brief This method checks on the STORM job of the shield.
   *
   * @param shield A reference to the Shield instance.
   * @return A Integer with the status (running==-1, succeeded==0, error==1).
   */
  int checkOnStorm(Shield *shield);
};

#endif //INCLUDE_STORMCONNECTOR_H_
