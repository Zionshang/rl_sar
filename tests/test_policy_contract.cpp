/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#include "test_support.hpp"
#include "fsm_unitree.hpp"

int main()
{
    try
    {
        for (const auto& item : std::vector<std::pair<std::string, std::string>>{
                 {"go2", "himloco"}, {"go2", "robot_lab"}, {"go2w", "robot_lab"}})
        {
            TestRobot robot;
            robot.Configure(item.first, item.second);
            auto obs = robot.ComputeObservation();
            require(obs.size() == (item.first == "go2" ? 45 : 57), "Observation size changed");
            auto history = robot.params.Get<std::vector<int>>("observations_history");
            if (!history.empty())
            {
                ObservationBuffer buffer(1, robot.obs_dims, 6, "time");
                buffer.insert(obs);
                require(buffer.get_obs_vec(history).size() == 270, "HIMLoco requires 270 input values");
            }
            auto count = robot.params.Get<int>("num_of_dofs");
            std::vector<float> positions, velocities, torques;
            robot.ComputeOutput(std::vector<float>(count, 1.0f), positions, velocities, torques);
            auto defaults = robot.params.Get<std::vector<float>>("default_dof_pos");
            auto scales = robot.params.Get<std::vector<float>>("action_scale");
            for (int i = 0; i < count; ++i)
            {
                bool wheel = i >= 12;
                require(std::abs(positions[i] - (defaults[i] + (wheel ? 0.0f : scales[i]))) < 1e-6f, "Position action mapping changed");
                require(velocities[i] == (wheel ? scales[i] : 0.0f), "Wheel velocity action mapping changed");
            }
            bool rejected = false;
            try { robot.ComputeOutput({1.0f}, positions, velocities, torques); }
            catch (const std::invalid_argument&) { rejected = true; }
            require(rejected, "Invalid action dimensions must be rejected");
            robot.fsm = CreateUnitreeFSM(robot);
            robot.fsm.RequestStateChange("RLFSMStateGetUp");
            for (int i = 0; i < 650; ++i) robot.StateController(&robot.robot_state, &robot.robot_command);
            require(robot.robot_command.motor_command.q.size() == static_cast<size_t>(count), "Standing command dimensions changed");
            robot.config_name = "missing_policy";
            robot.fsm.RequestStateChange("RLFSMStateRLLocomotion");
            robot.StateController(&robot.robot_state, &robot.robot_command);
            require(!robot.rl_init_done, "Failed initialization must not enable policy inference");
            robot.StateController(&robot.robot_state, &robot.robot_command);
            require(robot.fsm.current_state_->GetStateName() == "RLFSMStatePassive", "Failed policy must return to passive mode");
        }
        std::cout << "Go2/Go2W observations, historical input, wheel actions and FSM checks passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
