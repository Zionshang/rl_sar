#ifndef FSM_CORE_HPP
#define FSM_CORE_HPP

#include <iostream>
#include <string>
#include <unordered_map>
#include <memory>
#include <vector>
#include "logger.hpp"

class FSMState
{
public:
    FSMState(std::string name) : state_name_(std::move(name)) {}
    virtual ~FSMState() = default;

    virtual void Enter() = 0;
    virtual void Run() = 0;
    virtual void Exit() = 0;
    virtual std::string CheckChange() { return state_name_; }

    const std::string &GetStateName() const { return state_name_; }

protected:
    std::string state_name_;
};

class FSM
{
public:
    FSM() : current_state_(nullptr), next_state_(nullptr), previous_state_(nullptr), mode_(Mode::NORMAL) {}

    void AddState(std::shared_ptr<FSMState> state)
    {
        states_[state->GetStateName()] = state;
    }

    void SetInitialState(const std::string &name)
    {
        current_state_ = states_.at(name);
        current_state_->Enter();
        next_state_ = current_state_;
        std::cout << LOGGER::INFO << "[FSM] Set initial state: " << name << std::endl;
    }

    void RequestStateChange(const std::string& state_name)
    {
        if (states_.find(state_name) == states_.end())
        {
            std::cout << LOGGER::ERROR << "[FSM] State '" << state_name << "' not found!" << std::endl;
            return;
        }

        if (current_state_ && current_state_->GetStateName() != state_name)
        {
            next_state_ = states_.at(state_name);
            mode_ = Mode::CHANGE;
            std::cout << std::endl << LOGGER::INFO << "[FSM] Request switch from " << current_state_->GetStateName() << " to " << next_state_->GetStateName() << std::endl;
        }
    }

    void Run()
    {
        if (!current_state_)
            return;

        if (mode_ == Mode::NORMAL)
        {
            current_state_->Run();
            std::string next = current_state_->CheckChange();
            if (next != current_state_->GetStateName())
            {
                mode_ = Mode::CHANGE;
                next_state_ = states_.at(next);
                std::cout << std::endl << LOGGER::NOTE << "[FSM] Switch from " << current_state_->GetStateName() << " to " << next_state_->GetStateName() << std::endl;
            }
        }
        else if (mode_ == Mode::CHANGE)
        {
            current_state_->Exit();
            previous_state_ = current_state_;
            current_state_ = next_state_;
            auto expected_state = next_state_;  // Save the state we're entering
            current_state_->Enter();

            // Check if Enter() triggered another state change request
            if (mode_ == Mode::CHANGE && next_state_ != expected_state) {
                // Enter() requested a new state change, return and process it in next cycle
                return;
            }

            mode_ = Mode::NORMAL;
            current_state_->Run();
        }
    }

    enum class Mode
    {
        NORMAL,
        CHANGE
    };

    std::unordered_map<std::string, std::shared_ptr<FSMState>> states_;
    std::shared_ptr<FSMState> current_state_;
    std::shared_ptr<FSMState> next_state_;
    std::shared_ptr<FSMState> previous_state_;
    Mode mode_;
};

#endif // FSM_CORE_HPP
