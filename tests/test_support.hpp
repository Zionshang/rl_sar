/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "rl_sdk.hpp"
#include <cmath>
#include <stdexcept>

inline void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

class TestRobot : public RL
{
public:
    std::vector<float> Forward() override { return obs.actions; }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}

    void Configure(const std::string& robot, const std::string& policy)
    {
        robot_name = robot;
        config_name = policy;
        ReadYaml(robot, "base.yaml");
        ReadYaml(robot + "/" + policy, "config.yaml");
        InitJointNum(params.Get<int>("num_of_dofs"));
        InitObservations();
        InitOutputs();
    }
};
