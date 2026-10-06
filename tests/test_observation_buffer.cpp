/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#include "observation_buffer.hpp"
#include <iostream>
#include <stdexcept>

void expect(const std::vector<float>& actual, const std::vector<float>& expected)
{
    if (actual != expected) throw std::runtime_error("Historical observation order changed");
}

int main()
{
    try
    {
        ObservationBuffer time(1, {2, 1}, 6, "time");
        ObservationBuffer term(1, {2, 1}, 6, "term");
        for (auto* buffer : {&time, &term})
        {
            buffer->insert({1, 2, 3});
            buffer->insert({4, 5, 6});
            buffer->insert({7, 8, 9});
        }
        expect(time.get_obs_vec({0, 2}), {7, 8, 9, 1, 2, 3});
        expect(term.get_obs_vec({0, 2}), {7, 8, 1, 2, 9, 3});
        // Frame 5 is valid even though there are only two observation terms.
        expect(time.get_obs_vec({5}), {0, 0, 0});
        expect(term.get_obs_vec({5}), {0, 0, 0});
        expect(time.get_obs_vec({0, 0}), {7, 8, 9, 7, 8, 9});
        time.reset({0}, {10, 11, 12});
        expect(time.get_obs_vec({0, 5}), {10, 11, 12, 10, 11, 12});
        std::cout << "Historical observation ordering and reset checks passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
