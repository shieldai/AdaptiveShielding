#include <fstream>

#include "LaneTree.h"
#include "ISumo.h"
#include "Util.h"

LaneTree::LaneTree(ISumo *sumo, const std::string &lane, bool virt) : sumo(sumo) {
  root = buildTree(sumo, lane, 0, virt);
}

LaneTree::~LaneTree() {
  destroyTree(root);
}

void LaneTree::trackNode(int &vn, int &hn) {
  trackNode(sumo, root, vn, hn);
}

void LaneTree::logNode(std::ofstream &log) {
  logNode(log, root, 0);
}

treeNode *LaneTree::buildTree(ISumo *sumo, const std::string &rootLane, double deep, bool virt) {
  // NODE INFO
  auto length = sumo->getLaneLength(rootLane);
  auto max = length/DEFAULT_VEHICLE_LENGTH;

  auto *tree = new treeNode;
  tree->length = length;
  tree->label = virt ? sumo->virtualLaneID(rootLane) : rootLane;

  deep += max;
  // Recursive Function: termination condition
  if(deep >= gConfig.maxLaneSize) {
    return tree;
  }

  // BRANCHES
  std::vector<std::string> prevLanes = sumo->findIncomingLanes(rootLane);
  for(const auto &prev : prevLanes) {
    auto branch = buildTree(sumo, prev, deep, virt);
    tree->previous.insert(branch);
  }

  return tree;
}

void LaneTree::destroyTree(treeNode *tree) {
  // DESTROY BRANCHES
  for(const auto &branch : tree->previous) {
    destroyTree(branch);
  }

  // DESTROY NODE
  delete tree;
}

void LaneTree::trackNode(ISumo *sumo, treeNode *node, int &vn, int &hn) {

  auto v = sumo->getLaneLastStepVehicleNumber(node->label);
  auto h = sumo->getLaneLastStepHaltingNumber(node->label);

  // NOTE: for the most TCs the lanes are parallel!
  // For more complex scenarios, it would be necessary to model the lanes in a better
  // way like check lengths and lane positions.

  int parallelVehicles = 0;
  int parallelHaltings = 0;
  for(auto n : node->previous) {
    int vehicles = 0;
    int haltings = 0;
    trackNode(sumo, n, vehicles, haltings);

    parallelVehicles = std::max(parallelVehicles, vehicles);
    parallelHaltings = std::max(parallelHaltings, haltings);
  }

  // NOTE: The limiter should prevent that we count wrong for very long lanes.
  // If we have a very long lane size, we can grow in other traffic lights,
  // and therefore we want to count continuous lanes.
  if(v!=0 /*|| vn == 0*/) {
    vn += v + parallelVehicles;
  }

  if(h!=0 /*|| hn == 0*/) {
    hn += h + parallelHaltings;
  }
}

void LaneTree::logNode(std::ofstream &log, treeNode *node, int deep) {
  if(deep==0) {
    log << "------------------------------------------" << std::endl;
    log << "Tree ROOT " << node->label << ", lentgh " << node->length << ", veh " << std::endl;
  } else {
    for(int i = 0; i < deep; i++) {
      log << "\t";
    }
    log << "NODE " << node->label << ", lentgh " << node->length << std::endl;
  }

  deep++;
  for(auto n : node->previous) {
    logNode(log, n, deep);
  }

  if(deep==1) {
    log << "------------------------------------------" << std::endl;
  }
}

void LaneTree::getSumoLabelsInTree(std::vector<std::string> &labels) {
  getSumoLabelsInTree(root, labels);
}

void LaneTree::getSumoLabelsInTree(treeNode *node, std::vector<std::string> &labels) {
  labels.push_back(node->label);
  for(auto n : node->previous) {
    getSumoLabelsInTree(n, labels);
  }
}

void LaneTree::testLaneTrees(const std::set<std::set<LaneTree *>> &laneTrees) {

  // GLOBAL: ALL TREES OF THE TRAFFIC LIGHT
  std::map<std::string, int> counterGlobal;

  // MERGE: Check the label occurrence in a merged lane, here we have already multiple trees in parallel.
  std::map<std::string, int> counterMerge;

  // LOCAL: Check the label occurrence in one tree and detect cycles in the tree
  std::map<std::string, int> counterLocal;

  for(const auto &mergedLaneTrees : laneTrees) {
    counterMerge.clear();

    for(auto tree : mergedLaneTrees) {
      counterLocal.clear();

      std::vector<std::string> labels;
      tree->getSumoLabelsInTree(labels);

      for(const auto &label : labels) {
        counterLocal[label]++;
        counterMerge[label]++;
        counterGlobal[label]++;
      }

      for(const auto &label : labels) {
        if(counterLocal[label]!=1) {
          throw std::logic_error("Unsupported lane scenario.");
        }
        //assert(counterLocal[label]==1);
      }
    }

    for(const auto &p : counterMerge) {
      if(p.second > 1) {
        // std::cout << "DUP TREE " << p.first << " = " << p.second << std::endl;
      }
    }
  }

  for(const auto &p : counterGlobal) {
    if(p.second > 1) {
      // std::cout << "DUP GLOB " << p.first << " = " << p.second << std::endl;
    }
  }
}