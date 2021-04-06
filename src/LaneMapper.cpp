#include <algorithm>
#include <cassert>

#include "ISumo.h"
#include "LaneMapper.h"

LaneMapper::LaneMapper(ISumo *sumo, const std::string &tls_id) : sumo(sumo), tlsID(tls_id) {
  loadTrafficLightPhaseInfo(tls_id);
  initLaneMapper(tls_id);
  testLaneTrees();
}

LaneMapper::~LaneMapper() {
  for(const auto &hasLaneTrees : m) {
    for(auto tree : hasLaneTrees.second.sumoLabelsTree) {
      delete tree;
    }
  }
}

std::vector<std::string> LaneMapper::getLabels() {
  std::vector<std::string> labels;
  for(const auto &pair : m) {
    labels.push_back(pair.first);
  }
  return labels;
}

std::set<std::string> LaneMapper::getSumoLabel(const std::string &lane) {
  return m[lane].sumoLabels;
}

std::set<LaneTree *> LaneMapper::getSumoLabelTree(const std::string &lane) {
  return m[lane].sumoLabelsTree;
}

const std::vector<std::vector<std::string>> &LaneMapper::getFormattedLinks() {
  return formattedIncomingLanesPerPhase;
}

void LaneMapper::loadTrafficLightPhaseInfo(const std::string &tlsID) {
  //auto links = sumo->trafficlights.getControlledLinks(tlsID);
  auto links = sumo->getTrafficLightsControlledLinks(tlsID);

  //auto programLogics = sumo->trafficlights.getAllProgramLogics(tlsID);
  auto programLogics = sumo->getTrafficLightsAllProgramLogics(tlsID);
  auto tlsLogic = programLogics.at(0);

  if(!checkTrafficLightProgram(tlsLogic)) {
    throw std::logic_error("Unsupported traffic light scenario.");
  }

  std::vector<std::string> inLanes;
  // std::vector<std::string> inLaneEdges;
  for(const auto &link : links) {
    std::string inLane;
    for(const auto &connections : link) {
      if(inLane.empty()) {
        inLane = connections.fromLane;
      }
      assert(inLane==connections.fromLane);
    }

    inLanes.push_back(inLane);
    //auto edge = sumo->lane.getEdgeID(inLane);
    //inLaneEdges.push_back(edge);
  }

  /*
  assert(inLanes.size()==inLaneEdges.size());
  for(size_t i = 0; i < inLanes.size(); i++) {
    for(size_t j = 0; j < inLanes.size(); j++) {
      if(inLanes[i]==inLanes[j]) { assert(inLaneEdges[i]==inLaneEdges[j]); }
    }
  } */

  std::vector<std::vector<int>> greenByPhase;
  greenByPhase.resize(inLanes.size());
  for(size_t i = 0; i < tlsLogic.phases.size(); i++) {
    auto state = tlsLogic.phases[i]->state;
    assert(state.size()==inLanes.size());
    if(state.find('y')!=std::string::npos) {
      continue;
    }

    non_yellow++;

    for(size_t j = 0; j < inLanes.size(); j++) {
      if(state[j]=='g' || state[j]=='G') {
        greenByPhase[j].push_back(i);
      }
    }
  }

  // MERGE BY PHASES!
  std::map<std::string, std::vector<std::vector<int>>> mergeMap;
  std::vector<std::vector<int>> mergeSets;
  mergeSets.resize(inLanes.size());
  for(size_t i = 0; i < inLanes.size(); i++) {
    std::vector<int> greenInPhase = greenByPhase[i];
    std::sort(greenInPhase.begin(), greenInPhase.end());

    for(auto phase : greenInPhase) {
      laneControlledInPhase[inLanes[i]].insert(phase);
    }

    mergeMap[inLanes[i]].push_back(greenInPhase);
    for(size_t j = 0; j < inLanes.size(); j++) {
      if(i==j) {
        continue;
      }
      std::vector<int> greenInPhaseRef = greenByPhase[j];
      std::sort(greenInPhaseRef.begin(), greenInPhaseRef.end());
      if(greenInPhase==greenInPhaseRef) {
        mergeSets[i].push_back(j);
      } else {
        // assert(inLaneEdges[i] != inLaneEdges[j] && "Merge on Edge not possible!");
      }
    }
  }

  /*
  for(const auto &pair : mergeMap) {
    std::cout << "MERGE SET FOR " << pair.first << "(" << sumo->lane.getEdgeID(pair.first) << "): ";
    for(const auto &mergeSet : pair.second) {
      std::cout << "[";
      for(auto i : mergeSet) { std::cout << i << " "; }
      std::cout << "], ";
    }
    std::cout << std::endl;
  }*/
}

void LaneMapper::initLaneMapper(const std::string &tls_id) {
  auto links = sumo->getTrafficLightsControlledLinks(tls_id);

  std::vector<std::vector<std::string>> inLanes;
  for(auto &link : links) {
    std::vector<std::string> in;
    for(const auto &l : link) {
      //auto tree = new LaneTree(sumo_, l.fromLane);
      //laneMapper.logNode(log, tree, 0);
      auto formatedLane = insert(l.fromLane, !gConfig.noMerging, false);

      if(!formatedLane.empty()) {
        in.push_back(formatedLane);
      }

      if(sumo->hasVirtualLane(l.fromLane)) {
        std::cout << "Add virt lane to LaneMapper " << l.fromLane << std::endl;
        // needs a own tree with modified sumo labels
        //auto virtTree = new LaneTree(sumo_, l.fromLane, 0, true);

        // ADD VIRT LANE
        auto virtFormatedLane = insert(sumo->virtualLaneID(l.fromLane), false, true);
        if(!virtFormatedLane.empty()) {
          in.push_back(virtFormatedLane);
        }
      }
    }

    inLanes.push_back(in);
  }

  assert(inLanes.size()==links.size());
  formattedIncomingLanesPerPhase = inLanes;
}

void LaneMapper::testLaneTrees() {
  std::set<std::set<LaneTree *>> laneTrees;
  for(const auto &hasLaneTrees : m) {
    laneTrees.insert(hasLaneTrees.second.sumoLabelsTree);
  }
  LaneTree::testLaneTrees(laneTrees);
}

std::string LaneMapper::insert(const std::string &sumoLaneLabel, bool merge, bool virt) {
  std::string label = sumoLaneLabel;
  auto phases = laneControlledInPhase[sumoLaneLabel];

  // NOTE: Avoid the merge of parallel lanes which are controlled in different phases.
  // DO NOT merge lanes which are controlled in multiple phases,
  // if you do not check the phases are equal for all lanes.
  if(merge && phases.size()!=1 || virt) {
    merge = false;
  }

  // Check the green phases - if lane has green in every non yellow state it is always green!
  // NOTE: if the lane is always green we do not want it in our MDP!
  if((int)phases.size() >= non_yellow) {
    std::cerr << sumoLaneLabel << ": ALWAYS GREEN!" << std::endl;
    return "";
  }

  if(merge) {
    auto sp = split(sumoLaneLabel, "_");
    assert(sp.size()==2 && "LaneMapper::insert: A new name format occurred here, adaptions are needed.");
    label = sp[0];
    // auto edge = sumo->lane.getEdgeID(sumoLaneLabel);
    // assert(label==edge && "This happens the first time ...");
  }

  formatName(label);

  auto result = m[label].sumoLabels.insert(sumoLaneLabel);
  if(result.second) {
    auto tree = new LaneTree(sumo, sumoLaneLabel, virt);
    //laneMapper.logNode(log, tree, 0);
    m[label].sumoLabelsTree.insert(tree);
  }

  assert(m[label].sumoLabels.size()==m[label].sumoLabelsTree.size());

  return label;
}

bool LaneMapper::checkTrafficLightProgram(libsumo::TraCILogic tlsLogic) {
  std::vector<int> green(tlsLogic.phases[0]->state.size(), 0);
  int non_yellow = 0;

  for(auto state : tlsLogic.phases) {
    if(state->state.find('y')==std::string::npos) {
      non_yellow++;

      size_t count_green = 0;
      size_t count_red = 0;

      for(size_t i = 0; i < state->state.size(); i++) {
        auto c = state->state[i];
        if(c=='g' || c=='G') {
          green[i]++;
          count_green++;
        } else if(c=='r' || c=='R') {
          //red[i]++;
          count_red++;
        } else if(c=='y' || c=='Y') {
          //yellow[i]++;
          //count_yellow++;
        } else
          assert("HHMMM.. There is something new!");
      }

      // THIS CASES WE CAN NOT HANDLE ...
      if(count_green==state->state.size()) {
        std::cerr << tlsID << " state " << state->state << " ALL LANES GREEN!" << std::endl;
        return false;
      } else if(count_red==state->state.size()) {
        std::cerr << tlsID << " state " << state->state << " ALL LANES RED!" << std::endl;
        return false;
      }
    }
  }

  return true;
}







