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

#include <gtest/gtest.h>

#include <string>

#include "my_robot_grasp_attach/grasp_attach_command.hpp"

using my_robot_grasp_attach::AttachCommand;
using my_robot_grasp_attach::parse_attach_command;

TEST(ParseAttachCommand, ParsesTheFourNames)
{
  AttachCommand command;
  std::string error;
  ASSERT_TRUE(parse_attach_command(
    "attach mobile_manipulator panda_hand grasp_box link", command, error))
    << error;
  EXPECT_FALSE(command.detach);
  EXPECT_EQ(command.parent_model, "mobile_manipulator");
  EXPECT_EQ(command.parent_link, "panda_hand");
  EXPECT_EQ(command.child_model, "grasp_box");
  EXPECT_EQ(command.child_link, "link");
}

TEST(ParseAttachCommand, AcceptsSurroundingWhitespace)
{
  AttachCommand command;
  std::string error;
  ASSERT_TRUE(parse_attach_command(
    "  attach  robot  hand  box  link \n", command, error)) << error;
  EXPECT_EQ(command.child_model, "box");
}

TEST(ParseAttachCommand, ParsesDetach)
{
  AttachCommand command;
  std::string error;
  ASSERT_TRUE(parse_attach_command("detach", command, error)) << error;
  EXPECT_TRUE(command.detach);
}

TEST(ParseAttachCommand, RejectsDetachWithArguments)
{
  AttachCommand command;
  std::string error;
  EXPECT_FALSE(parse_attach_command("detach now", command, error));
  EXPECT_FALSE(error.empty());
}

TEST(ParseAttachCommand, RejectsAnEmptyCommand)
{
  AttachCommand command;
  std::string error;
  EXPECT_FALSE(parse_attach_command("", command, error));
  EXPECT_FALSE(parse_attach_command("   ", command, error));
  EXPECT_FALSE(error.empty());
}

TEST(ParseAttachCommand, RejectsUnknownVerbs)
{
  AttachCommand command;
  std::string error;
  EXPECT_FALSE(parse_attach_command("grab robot hand box link", command, error));
  EXPECT_NE(error.find("grab"), std::string::npos);
}

TEST(ParseAttachCommand, RejectsTheWrongNumberOfNames)
{
  AttachCommand command;
  std::string error;
  EXPECT_FALSE(parse_attach_command(
    "attach mobile_manipulator panda_hand grasp_box", command, error));
  EXPECT_FALSE(parse_attach_command(
    "attach mobile_manipulator panda_hand grasp_box link extra", command, error));
  EXPECT_FALSE(error.empty());
}
