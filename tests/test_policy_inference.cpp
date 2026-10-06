/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#include "test_support.hpp"

int main()
{
    try
    {
#ifdef USE_TORCH
        for (const auto& item : std::vector<std::pair<std::string, std::string>>{
                 {"go2", "himloco"}, {"go2", "robot_lab"}, {"go2w", "robot_lab"}})
        {
            TestRobot robot;
            robot.robot_name = item.first;
            robot.ReadYaml(item.first, "base.yaml");
            robot.InitRL(item.first + "/" + item.second);
            auto input = robot.ComputeObservation();
            auto history = robot.params.Get<std::vector<int>>("observations_history");
            if (!history.empty())
            {
                robot.history_obs_buf.insert(input);
                input = robot.history_obs_buf.get_obs_vec(history);
            }
            auto first = robot.model->forward({input});
            require(first.size() == static_cast<size_t>(robot.params.Get<int>("num_of_dofs")), "Torch output dimensions changed");
            for (float value : first) require(std::isfinite(value), "Nonfinite Torch action");
            require(first == robot.model->forward({input}), "Policy inference must be deterministic");
        }
#endif
#ifdef USE_ONNX
        const std::vector<float> input = {-1.0f, 2.0f, 3.0f};
        for (const auto& filename : {"identity_dynamic.onnx", "identity_fixed.onnx"})
        {
            auto path = std::string(TEST_FIXTURE_DIR) + "/" + filename;
            auto model = InferenceRuntime::ModelFactory::load_model(path);
            require(model && model->get_model_type() == "onnx", "ONNX backend unavailable");
            require(model->forward({input}) == input, "ONNX inference changed input values");
            require(model->load(path), "ONNX reload failed");
            require(model->forward({input}) == input, "ONNX reload corrupted node names");
        }
        auto model = InferenceRuntime::ModelFactory::load_model(std::string(TEST_FIXTURE_DIR) + "/identity_fixed.onnx");
        bool rejected = false;
        try { model->forward({{1.0f, 2.0f}}); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "ONNX must reject input shape mismatches");
#endif
        std::cout << "Enabled inference backends passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
