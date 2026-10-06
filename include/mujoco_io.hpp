/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "rl_sdk.hpp"
#include <mujoco/mujoco.h>

inline void ValidateMujocoModel(const mjModel& model, const YamlParams& params)
{
    const int count = params.Get<int>("num_of_dofs");
    const auto mapping = params.Get<std::vector<int>>("joint_mapping");
    if (model.nu != count || model.nsensordata < 3 * count + 7 || mapping.size() != static_cast<size_t>(count))
        throw std::invalid_argument("MuJoCo actuators/sensors do not match the robot configuration");
    std::vector<bool> seen(count, false);
    for (int index : mapping)
    {
        if (index < 0 || index >= count || seen[index])
            throw std::invalid_argument("joint_mapping must be a permutation of the actuator indices");
        seen[index] = true;
    }
    const int quat = mj_name2id(&model, mjOBJ_SENSOR, "imu_quat");
    const int gyro = mj_name2id(&model, mjOBJ_SENSOR, "imu_gyro");
    if (quat < 0 || gyro < 0 || model.sensor_adr[quat] != 3 * count || model.sensor_dim[quat] != 4 ||
        model.sensor_adr[gyro] != 3 * count + 4 || model.sensor_dim[gyro] != 3)
        throw std::invalid_argument("MuJoCo IMU sensor offsets do not match the controller");
}

inline void ReadMujocoState(const mjData& data, const YamlParams& params, RobotState<float>& state)
{
    const int count = params.Get<int>("num_of_dofs");
    const auto mapping = params.Get<std::vector<int>>("joint_mapping");
    for (int i = 0; i < 4; ++i) state.imu.quaternion[i] = data.sensordata[3 * count + i];
    for (int i = 0; i < 3; ++i) state.imu.gyroscope[i] = data.sensordata[3 * count + 4 + i];
    for (int i = 0; i < count; ++i)
    {
        state.motor_state.q[i] = data.sensordata[mapping[i]];
        state.motor_state.dq[i] = data.sensordata[count + mapping[i]];
        state.motor_state.tau_est[i] = data.sensordata[2 * count + mapping[i]];
    }
}

inline void WriteMujocoCommand(mjData& data, const YamlParams& params, const RobotCommand<float>& command)
{
    const int count = params.Get<int>("num_of_dofs");
    const auto mapping = params.Get<std::vector<int>>("joint_mapping");
    const auto& motor = command.motor_command;
    for (int i = 0; i < count; ++i)
    {
        data.ctrl[mapping[i]] = motor.tau[i] +
            motor.kp[i] * (motor.q[i] - data.sensordata[mapping[i]]) +
            motor.kd[i] * (motor.dq[i] - data.sensordata[count + mapping[i]]);
    }
}
