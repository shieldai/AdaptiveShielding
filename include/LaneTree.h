#ifndef INCLUDE_LANETREE_H_
#define INCLUDE_LANETREE_H_

#include <vector>
#include <string>
#include <set>

class ISumo;

/**
 * Struct treeNode. The Basic structure to build a tree.
 */
struct treeNode {
  std::string label{};
  double length{};
  std::set<treeNode *> previous;
};

/** @class LaneTree
 * Contains a tree of lanes to reconstruct and track the path away from a lane.
 *
 * The TraCI API does not provide any possibilities to back from one lane.
 * Therefore we build an internal tree structure to track the interesting properties manually.
 */
class LaneTree {
 private:
  ISumo *sumo;
  treeNode *root;

 public:
  /** @brief Constructor which calls a static function to build a tree starting with the given start lane.
 *
 * @param sumo  Pointer to a SUMOConnector instance
 * @param lane  String of the start lane
 * @param virt  Boolean flag marks a tree for a virtual lane
 */
  LaneTree(ISumo *sumo, const std::string &lane, bool virt = false);

  /// @brief Destructor for the class, calls a static function to destroy the tree.
  ~LaneTree();

  /** @brief The method starts the call for the recursive function to track the properties of the tree lanes.
   *
   * @param[out] vn  Returns the vehicle number.
   * @param[out] hn  Returns the halting number.
   */
  void trackNode(int &vn, int &hn);

  /** @brief The method starts the call for the recursive function to log the tree.
   *
   * @param log  Reference to a ofstream instance.
   */
  void logNode(std::ofstream &log);

  /** @brief DEBUG function to find conflicts in all trees of one Traffic Light.
   * The function asserts if we detect a cycle in a tree.
   * For other scenarios, we only print a warning.
   *
   * @param laneTrees  Set of set with all tress of one traffic light.
   */
  static void testLaneTrees(const std::set<std::set<LaneTree *>> &laneTrees);

 private:
  /** @brief Recursive Function to build the lane tree
   * the termination condition is given by the global variable gConfig.maxLaneSize.
   *
   * @param sumo  Pointer to a SUMOConnector instance.
   * @param lane  String of the start lane.
   * @param deep  Length of the branch with parent nodes.
   * @param virt  Boolean flag marks a tree for a virtual lane.
   * @return the root node of the tree.
   */
  static treeNode *buildTree(ISumo *sumo, const std::string &rootLane, double deep = 0., bool virt = false);

  /** @brief The function gets the root node of the tree and destroys all branches in the tree.
   *
   * @param tree  Pointer to a tree node.
   */
  static void destroyTree(treeNode *tree);

  /** @brief Recursive Function to track the vicle number and halting vehicle number of the lane tree.
   *
   * @param sumo  Pointer to a SUMOConnector instance.
   * @param node  Pointer to a tree node.
   * @param[out] vn  Returns the vehicle number.
   * @param[out] hn  Returns the halting number.
   */
  static void trackNode(ISumo *sumo, treeNode *node, int &vn, int &hn);

  /** @brief Recursive Function to log the lane tree.
   *
   * @param log  Reference to a ofstream instance.
   * @param node  Pointer to a tree node.
   * @param deep  Level of the node.
   */
  void logNode(std::ofstream &log, treeNode *node, int deep);

  /** @brief The method starts the call for the recursive function to collect the lane labels in the tree.
   *
   * @param[out] labels  Reference to a vector which will be filled with the lane labes.
   */
  void getSumoLabelsInTree(std::vector<std::string> &labels);;

  /** @brief Recursive Function to collect all lane labels in the tree.
   *
   * @param node  Pointer to a tree node.
   * @param[out] labels  Reference to a vector which will be filled with the lane labes.
   */
  void getSumoLabelsInTree(treeNode *node, std::vector<std::string> &labels);
};

#endif //INCLUDE_LANETREE_H_
