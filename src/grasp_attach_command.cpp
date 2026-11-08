// Copyright 2026 robot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "my_robot_grasp_attach/grasp_attach_command.hpp"

#include <sstream>
#include <string>
#include <vector>

namespace my_robot_grasp_attach
{

bool parse_attach_command(
  const std::string & text, AttachCommand & command, std::string & error)
{
  std::istringstream stream(text);
  std::vector<std::string> tokens;
  std::string token;
  while (stream >> token) {
    tokens.push_back(token);
  }

  if (tokens.empty()) {
    error = "empty command";
    return false;
  }

  if (tokens[0] == "detach") {
    if (tokens.size() != 1) {
      error = "'detach' takes no arguments";
      return false;
    }
    command = AttachCommand();
    command.detach = true;
    error.clear();
    return true;
  }

  if (tokens[0] != "attach") {
    error = "unknown command '" + tokens[0] + "'";
    return false;
  }

  if (tokens.size() != 5) {
    error =
      "expected 'attach <parent_model> <parent_link> <child_model> "
      "<child_link>'";
    return false;
  }

  command = AttachCommand();
  command.parent_model = tokens[1];
  command.parent_link = tokens[2];
  command.child_model = tokens[3];
  command.child_link = tokens[4];
  command.detach = false;
  error.clear();
  return true;
}

}  // namespace my_robot_grasp_attach
