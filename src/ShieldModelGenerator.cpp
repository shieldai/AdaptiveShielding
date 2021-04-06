#include "ShieldModelGenerator.h"
#include "Environment.h"
#include "Controller.h"
#include "Util.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <utility>

ShieldModelGenerator::ShieldModelGenerator(const std::string &filenamePrefix) :
    filenamePrefix(std::move(filenamePrefix)) {}

void ShieldModelGenerator::createPropFile() {
  std::ofstream PROPS(out_path_ + filenamePrefix + ".props", std::ios::trunc);
  PROPS << "// " << filenamePrefix + ".props" << " Created at " << getTimeString() << "\n\n";
  for(const auto &prop : properties) {
    PROPS << prop << "\n";
  }
  PROPS.close();
}

void ShieldModelGenerator::createPRISMFile(const Environment &environment, const Controller &controller) {
  std::ofstream PRISM(out_path_ + filenamePrefix + ".prism", std::ios::trunc);

  auto stateSpaceLabels = environment.getStateSpaceLabels();
  auto stateSpace = environment.getStateSpace();
  auto stateWeights = environment.getWeights();
  auto stateProbabilities = environment.getProbabilities();

  auto actionStateLabels = controller.getActionSpaceLabels();
  auto actionSpace = controller.getActionSpace();
  auto actionProbabilities = controller.getProbabilities();

  PRISM << "// " << filenamePrefix + ".prism" << " Created at " << getTimeString() << "\n\n";
  PRISM << modelType_ << "\n\n";

  for(size_t i = 0; i < stateSpaceLabels.size(); i++) {
    PRISM << "const double " << stateSpaceLabels.at(i) << "Prob = " <<
          std::fixed << std::setprecision(6) << stateProbabilities.at(i) << ";\n";
  }
  PRISM << "\n";

  for(size_t i = 0; i < stateSpaceLabels.size(); i++) {
    PRISM << "const int " << stateSpaceLabels.at(i) << "Max = " << stateSpace.at(i) << ";\n";
  }
  PRISM << "\n";

  for(size_t i = 0; i < actionProbabilities.size(); i++) {
    PRISM << "const double " << actionStateLabels.at(i) << "Prob = " << actionProbabilities.at(i) << ";\n";
  }
  PRISM << "\n";

  PRISM << "module arbiter\n";
  PRISM << "\tmove : [0 .. 2] init 0;\n";
  for(const std::string &line : module_arbiter)
    PRISM << "\t" << line << "\n";
  PRISM << "endmodule\n\n";

  PRISM << "module controller\n";
  PRISM << "\taction : [0 .. " << actionSpace.size() - 1 << "] init 0;\n";
  for(const std::string &line : module_controller)
    PRISM << "\t" << line << "\n";
  PRISM << "endmodule\n\n";

  PRISM << "module shield\n";
  for(size_t i = 0; i < stateSpaceLabels.size(); i++) {
    PRISM << "\t" << stateSpaceLabels.at(i) << ": [0 .. " << stateSpace.at(i) << "] init 0;\n";
  }
  PRISM << "\n";

  for(const std::string &line : module_environment)
    PRISM << "\t" << line << "\n";
  PRISM << "\n";

  for(const std::string &line : module_shield)
    PRISM << "\t" << line << "\n";
  PRISM << "endmodule\n\n";

  PRISM << "rewards\n";
  for(const std::string &reward : module_rewards)
    PRISM << "\t" << reward << "\n";
  PRISM << "\n";

  PRISM << "endrewards\n\n";
}

void ShieldModelGenerator::createPRISMArbiter(const Controller &controller) {
  std::vector<std::string> out;
  out.push_back("[env]    (move = 0) -> 1:(move' = 1);");
  out.push_back("[ctrl]   (move = 1) -> 1:(move' = 2);");
  for(const auto &l : controller.getActionSpaceLabels()) {
    out.push_back("[" + l + "] (move = 2) -> 1:(move' = 0);");
  }
  module_arbiter = out;
}

void ShieldModelGenerator::createPRISMEnvironment(const Environment &environment) {
  std::vector<std::string> out;
  out.push_back("[env] (true) ->");
  for(const auto &l : environment.getStateSpaceLabels()) {
    std::string line = l + "Prob : (" + l + "'=min(" + l + " + 1, " + l + "Max)) +";
    out.push_back(line);
  }
  out.back().back() = ';';
  module_environment = out;
}

void ShieldModelGenerator::createPRISMController(const Controller &controller) {
  std::vector<std::string> out;
  auto actionSpaceLabels = controller.getActionSpaceLabels();
  std::string line = "[ctrl] (true) -> ";
  for(size_t i = 0; i < actionSpaceLabels.size(); i++) {
    line += actionSpaceLabels[i] + "Prob : (action'=" + std::to_string(i) + ") +";
  }
  line.back() = ';';
  out.push_back(line);
  module_controller = out;
}

void ShieldModelGenerator::createPRISMShield(const Controller &controller) {
  std::vector<std::string> out;
  auto actionSpaceLabels = controller.getActionSpaceLabels();
  auto actionSpaceWay = controller.getActionSpaceWays();
  for(size_t i = 0; i < actionSpaceLabels.size(); i++) {
    for(size_t j = 0; j < actionSpaceLabels.size(); j++) {
      std::string line = "[" + actionSpaceLabels[i] + "]";
      line += " action=" + std::to_string(j) + " -> ";
      line += "0.9 : ";
      for(const auto &w : actionSpaceWay[i]) {
        line += "(" + w + "'=max(0, " + w + " - 1)) &";
      }
      line.back() = '+';
      line += " 0.1 : true;";
      out.push_back(line);
    }
  }
  module_shield = out;
}

void ShieldModelGenerator::createPRISMRewards(const Controller &controller) {
  std::vector<std::string> out;
  auto actionSpaceLabels = controller.getActionSpaceLabels();
  for(size_t i = 0; i < actionSpaceLabels.size(); i++) {
    for(size_t j = 0; j < actionSpaceLabels.size(); j++) {
      if(i!=j) {
        std::string line = "[" + actionSpaceLabels[i] + "]";
        line += " action=" + std::to_string(j) + " : " + std::to_string(gConfig.d) + ";";
        out.push_back(line);
      }
    }
  }

  auto ways = controller.getActionSpaceWays();
  std::vector<std::string> maxWays;
  for(size_t i = 0; i < actionSpaceLabels.size(); i++) {
    std::string line;
    if(ways[i].size()==1) {
      line = ways[i][0];
    } else {
      line = "max(";
      for(const auto &w : ways[i]) {
        line += w + ",";
      }
      line.back() = ')';
    }
    maxWays.push_back(line);
  }

  for(size_t i = 0; i < actionSpaceLabels.size(); i++) {
    for(size_t j = 0; j < actionSpaceLabels.size(); j++) {
      std::string line = "[" + actionSpaceLabels[i] + "]";
      line += " action=" + std::to_string(j) + " : 1 * max(";
      for(const auto &m1 : maxWays) {
        for(const auto &m2 : maxWays) {
          if(m1!=m2) {
            line += "(" + m1 + "-" + m2 + "),";
          }
        }
      }

      line.back() = ')';
      line += ";";
      out.push_back(line);
    }
  }

  module_rewards = out;
}

void ShieldModelGenerator::setModelType(std::string modelType) {
  modelType_ = modelType;
}

void ShieldModelGenerator::setProperties(std::vector<std::string> module) {
  properties = module;
}

void ShieldModelGenerator::setPRISMArbiter(const std::vector<std::string> &module) {
  module_arbiter = module;
}

void ShieldModelGenerator::setPRISMEnvironment(const std::vector<std::string> &module) {
  module_environment = module;
}

void ShieldModelGenerator::setPRISMController(const std::vector<std::string> &module) {
  module_controller = module;
}

void ShieldModelGenerator::setPRISMShield(const std::vector<std::string> &module) {
  module_shield = module;
}

void ShieldModelGenerator::setPRISMRewards(const std::vector<std::string> &module) {
  module_rewards = module;
}
