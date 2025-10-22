/*
 * Copyright 2025 IDRA, University of Trento
 * Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#ifndef UNITREE_Z1_HW_INTERFACE_HPP__
#define UNITREE_Z1_HW_INTERFACE_HPP__

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "unitree_arm_sdk/control/unitreeArm.h"

namespace unitree::z1 {

class HardwareInterface : public hardware_interface::SystemInterface {
public:
    using Vec6   = Eigen::Vector<double, 6>;
    using ArmPtr = std::unique_ptr<UNITREE_ARM::unitreeArm>;

    RCLCPP_SHARED_PTR_DEFINITIONS(HardwareInterface)

    HardwareInterface()           = default;
    ~HardwareInterface() override = default;

    HardwareInterface(const HardwareInterface&)             = delete;
    HardwareInterface(const HardwareInterface&&)            = delete;
    HardwareInterface& operator=(const HardwareInterface&)  = delete;
    HardwareInterface& operator=(const HardwareInterface&&) = delete;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    [[nodiscard]] std::vector<hardware_interface::ComponentInfo>& joints() { return info_.joints; }
    [[nodiscard]] const std::vector<hardware_interface::ComponentInfo>& joints() const { return info_.joints; }

    [[nodiscard]] rclcpp::Logger& get_logger() { return _logger; }
    [[nodiscard]] bool with_gripper() const;

private:
    rclcpp::Logger _logger = rclcpp::get_logger("z1_hardware_interface");
    ArmPtr _arm = nullptr;

    Vec6 _arm_max_torque     = 20.0 * Vec6::Ones();
    double _gripper_max_torque = 20.0;

    struct GainsData {
        std::vector<double> kp = {20.0, 30.0, 30.0, 20.0, 15.0, 10.0, 20.0};
        std::vector<double> kd = {2000, 2000, 2000, 2000, 2000, 2000, 2000};
    };

    GainsData _default_gains;
    GainsData _current_gains;

    struct {
        Vec6 q = Vec6::Zero();
        Vec6 qd = Vec6::Zero();
        Vec6 tau = Vec6::Zero();
    } _arm_state;

    struct {
        double q = 0;
        double qd = 0;
        double tau = 0;
    } _gripper_state;

    struct {
        Vec6 q = Vec6::Zero();
        Vec6 qd = Vec6::Zero();
        Vec6 tau = Vec6::Zero();
    } _arm_cmd;

    struct {
        double q = 0;
        double qd = 0;
        double tau = 0;
    } _gripper_cmd;

    void saturate_torque();
    long get_joint_id(const std::string& joint_name) const;

    // =======================================================================
    // Cambios para multithreading / buffers compartidos
    struct SharedState {
        Vec6 q, qd, tau;
        double g_q{0.0}, g_qd{0.0}, g_tau{0.0};
    };

    struct SharedCmd {
        Vec6 q, qd, tau;
        double g_q{0.0}, g_qd{0.0}, g_tau{0.0};
        std::array<double,7> kp{}, kd{};
    };

    SharedState state_buf_;
    SharedCmd cmd_buf_;
    std::mutex state_mtx_;
    std::mutex cmd_mtx_;
    std::atomic<bool> io_running_{false};
    std::thread io_thread_;
    std::atomic<uint64_t> cmd_seq_{0};

    void ioLoop();
    void stopIoThread();
};

}  // namespace unitree::z1

#endif  // UNITREE_Z1_HW_INTERFACE_HPP__