/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#include "test_support.hpp"
#include "mujoco_io.hpp"

int main()
{
    try
    {
        for (const auto& name : {"go2", "go2w"})
        for (const auto& scene : {"scene", "scene_terrain"})
        {
            TestRobot robot;
            robot.Configure(name, "robot_lab");
            auto filename = std::string(ASSET_DIR) + "/" + name + "/mjcf/" + scene + ".xml";
            char error[1024] = {};
            std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                mj_loadXML(filename.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
            require(model != nullptr, "Cannot load " + filename + ": " + error);
            ValidateMujocoModel(*model, robot.params);
            std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
            require(data != nullptr, "Cannot allocate MuJoCo data");
            mj_forward(model.get(), data.get());
            ReadMujocoState(*data, robot.params, robot.robot_state);
#ifdef USE_TORCH
            robot.InitRL(std::string(name) + "/robot_lab");
#endif
            for (int step = 0; step < 500; ++step)
            {
                ReadMujocoState(*data, robot.params, robot.robot_state);
                robot.obs.base_quat = robot.robot_state.imu.quaternion;
                robot.obs.ang_vel = robot.robot_state.imu.gyroscope;
                robot.obs.dof_pos = robot.robot_state.motor_state.q;
                robot.obs.dof_vel = robot.robot_state.motor_state.dq;
                if (step % robot.params.Get<int>("decimation") == 0)
                {
#ifdef USE_TORCH
                    robot.obs.actions = robot.RL::Forward();
#endif
                    robot.ComputeOutput(robot.obs.actions, robot.output_dof_pos, robot.output_dof_vel, robot.output_dof_tau);
                    robot.robot_command.motor_command.q = robot.output_dof_pos;
                    robot.robot_command.motor_command.dq = robot.output_dof_vel;
                    robot.robot_command.motor_command.kp = robot.params.Get<std::vector<float>>("rl_kp");
                    robot.robot_command.motor_command.kd = robot.params.Get<std::vector<float>>("rl_kd");
                }
                WriteMujocoCommand(*data, robot.params, robot.robot_command);
                mj_step(model.get(), data.get());
                for (int i = 0; i < model->nq; ++i) require(std::isfinite(data->qpos[i]), "Nonfinite MuJoCo joint position");
            }
            std::cout << name << "/" << scene << ": MJCF/resources and 500 control/physics steps passed\n";
        }
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
