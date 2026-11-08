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

#ifndef MY_ROBOT_GRASP_ATTACH__GRASP_ATTACH_COMMAND_HPP_
#define MY_ROBOT_GRASP_ATTACH__GRASP_ATTACH_COMMAND_HPP_

#include <string>

namespace my_robot_grasp_attach
{

/// One parsed command from the /grasp_attach topic.
struct AttachCommand
{
  std::string parent_model;
  std::string parent_link;
  std::string child_model;
  std::string child_link;
  bool detach = false;
};

/// Parse "attach <parent_model> <parent_link> <child_model> <child_link>"
/// or "detach".
/**
 * \param[in] text Message payload.
 * \param[out] command Parsed command; only valid when the call returns true.
 * \param[out] error Human readable reason when the call returns false.
 * \return true on success.
 */
bool parse_attach_command(
  const std::string & text, AttachCommand & command, std::string & error);

}  // namespace my_robot_grasp_attach

#endif  // MY_ROBOT_GRASP_ATTACH__GRASP_ATTACH_COMMAND_HPP_
