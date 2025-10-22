/*
 * Copyright 2025 IDRA, University of Trento
 * Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "z1_hardware_interface/z1_hardware_interface.hpp"

#include <algorithm>
#include <ctime>
#include <fmt/format.h>
#include <memory>
#include <stdexcept>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <unitree_arm_sdk/message/arm_common.h>

#include <rclcpp/duration.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

// CHANGE: multithreading - @JhonGonzalezR


//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

using unitree::z1::HardwareInterface;

static void to_lower_string(std::string& str);

template <typename Iterable>
static std::string
pretty_vector(const Iterable& vec) {
    return fmt::format("({})", fmt::join(vec, ", "));
}

static std::pair<std::string, std::string> split_interface(const std::string&);

// ===========================================================================
// CAMBIO: estructuras compartidas para estado y comando
// struct SharedState {
//   Vec6 q; Vec6 qd; Vec6 tau;
//   double g_q{0.0}, g_qd{0.0}, g_tau{0.0};
// };

// struct SharedCmd {
//   Vec6 q; Vec6 qd; Vec6 tau;
//   double g_q{0.0}, g_qd{0.0}, g_tau{0.0};
//   std::array<double,7> kp{}, kd{};
// };

// ===========================================================================

//  ____   ____ _     ____ ____  ____    _     _  __       ____           _
// |  _ \ / ___| |   / ___|  _ \|  _ \  | |   (_)/ _| ___ / ___|   _  ___| | ___
// | |_) | |   | |  | |   | |_) | |_) | | |   | | |_ / _ \ |  | | | |/ __| |/ _ \
// |  _ <| |___| |__| |___|  __/|  __/  | |___| |  _|  __/ |__| |_| | (__| |  __/
// |_| \_\\____|_____\____|_|   |_|     |_____|_|_|  \___|\____\__, |\___|_|\___|
//                                                             |___/
hardware_interface::CallbackReturn
HardwareInterface::on_configure(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_configure()");
    if (hardware_interface::SystemInterface::on_configure(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_configure() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

#ifdef SHOW_DEBUG_MESSAGES
    rclcpp::Logger logger = get_logger();
    logger.set_level(rclcpp::Logger::Level::Debug);
#endif

    if (with_gripper()) RCLCPP_INFO(get_logger(), "Gripper is enabled");
    else RCLCPP_INFO(get_logger(), "Gripper is disabled");

    // TODO: load torque limits from URDF
    RCLCPP_INFO(
            get_logger(),
            "Joint torque limits: %s",
            pretty_vector(_arm_max_torque).c_str()
    );
    RCLCPP_INFO(get_logger(), "Gripper torque limit: %lf", _gripper_max_torque);

    RCLCPP_INFO(get_logger(), "Establishing connection to the ARM through SDK");
    _arm = std::make_unique<UNITREE_ARM::unitreeArm>(with_gripper());
    RCLCPP_INFO(get_logger(), "Connection established!");
    _arm->sendRecvThread->start();
    _arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    // read(rclcpp::Time(0), rclcpp::Duration(0, 0));


    for(int i = 0; i < 5; ++i){
        _arm->sendRecv();
        for(int j=0;j<6;j++){
            _arm_state.q(j)   = _arm->lowstate->q[j];
            _arm_state.qd(j)  = _arm->lowstate->dq[j];
            _arm_state.tau(j) = _arm->lowstate->tau[j];
        }
        if(with_gripper()){
            _gripper_state.q   = _arm->lowstate->q[6];
            _gripper_state.qd  = _arm->lowstate->dq[6];
            _gripper_state.tau = _arm->lowstate->tau[6];
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // clang-format off
    RCLCPP_INFO(get_logger(), "Current joints configuration: %s", pretty_vector(_arm_state.q).c_str());
    RCLCPP_INFO(get_logger(), "Current joints velocity: %s", pretty_vector(_arm_state.qd).c_str());
    RCLCPP_INFO(get_logger(), "Measured joint torque: %s", pretty_vector(_arm_state.tau).c_str());
    RCLCPP_INFO(get_logger(), "Position-proportional gains: %s", pretty_vector(_default_gains.kp).c_str());
    RCLCPP_INFO(get_logger(), "Velocity-proportional gains: %s", pretty_vector(_default_gains.kd).c_str());
    // clang-format on

    // Set command to current state
    _arm_cmd.q      = _arm_state.q;
    _arm_cmd.qd     = _arm_state.qd;
    _gripper_cmd.q  = _gripper_state.q;
    _gripper_cmd.qd = _gripper_state.qd;
    _arm->setArmCmd(_arm_state.q, _arm_state.qd);
    _arm->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
    RCLCPP_INFO(get_logger(), "SDK switch to low-level control!");

    // ===========================================================================
  // CAMBIO: inicializar buffers
    {
        std::scoped_lock lk(state_mtx_, cmd_mtx_);
        for (int i=0;i<6;++i) {
            state_buf_.q(i)   = _arm_state.q(i);
            state_buf_.qd(i)  = _arm_state.qd(i);
            state_buf_.tau(i) = _arm_state.tau(i);
            cmd_buf_.q(i)     = _arm_cmd.q(i);
            cmd_buf_.qd(i)    = _arm_cmd.qd(i);
            cmd_buf_.tau(i)   = _arm_cmd.tau(i);
            cmd_buf_.kp[i]    = _current_gains.kp[i];
            cmd_buf_.kd[i]    = _current_gains.kd[i];
        }
        if (with_gripper()) {
            state_buf_.g_q   = _gripper_state.q;
            state_buf_.g_qd  = _gripper_state.qd;
            state_buf_.g_tau = _gripper_state.tau;
            cmd_buf_.g_q     = _gripper_cmd.q;
            cmd_buf_.g_qd    = _gripper_cmd.qd;
            cmd_buf_.g_tau   = _gripper_cmd.tau;
            cmd_buf_.kp[6]   = _current_gains.kp[6];
            cmd_buf_.kd[6]   = _current_gains.kd[6];
        }

    // ===========================================================================
    }

    RCLCPP_DEBUG(get_logger(), "on_configure() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_cleanup(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_cleanup()");
    if (hardware_interface::SystemInterface::on_cleanup(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_cleanup() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_cleanup() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_shutdown(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_shutdown()");
    if (hardware_interface::SystemInterface::on_shutdown(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_shutdown() failed");
    }

    //  Detener hilo IO de manera segura
    io_running_.store(false);
    if (io_thread_.joinable()) {
        RCLCPP_INFO(get_logger(), "Waiting for IO thread to finish...");
        io_thread_.join();
        RCLCPP_INFO(get_logger(), "IO thread finished.");
    }

    //  Cambiar a modo PASSIVE antes de mover brazo
    RCLCPP_INFO(get_logger(), "Setting arm into PASSIVE state");
    _arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // asegurar paquete recibido

    //  Llevar brazo a posición inicial (home)
    RCLCPP_INFO(get_logger(), "Returning arm to start position");
    _arm->backToStart();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // esperar movimiento

    // _arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    //  Cerrar SDK de manera segura
    RCLCPP_INFO(get_logger(), "Shutting down SDK connection");
    if (_arm->sendRecvThread)
        _arm->sendRecvThread->shutdown();

    RCLCPP_DEBUG(get_logger(), "on_shutdown() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_activate(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_activate()");
    if (hardware_interface::SystemInterface::on_activate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_shutdown() failed");
    }
    // TODO

    io_running_.store(true);
    io_thread_ = std::thread([this]{ this->ioLoop(); }); // CAMBIO: lanzar hilo IO


    RCLCPP_DEBUG(get_logger(), "on_activate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should deactivate the hardware.
 */
hardware_interface::CallbackReturn
HardwareInterface::on_deactivate(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_deactivate()");
    if (hardware_interface::SystemInterface::on_deactivate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_deactivate() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO


    stopIoThread();


    RCLCPP_DEBUG(get_logger(), "on_deactivate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should handle errors.
 */
hardware_interface::CallbackReturn
HardwareInterface::on_error(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "called on_error()");
    if (hardware_interface::SystemInterface::on_error(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_error() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_error() processed correctly");
    return hardware_interface::CallbackReturn::SUCCESS;
}

//  _   ___        __  ___       _             __
// | | | \ \      / / |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
// | |_| |\ \ /\ / /   | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
// |  _  | \ V  V /    | || | | | ||  __/ |  |  _| (_| | (_|  __/
// |_| |_|  \_/\_/    |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
//

std::vector<hardware_interface::StateInterface>
HardwareInterface::export_state_interfaces() {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(21);  // NOLINT: 7 joints * 3 states
    for (long i = 0; i < 6; ++i) {
        const std::string jnt_name = joints()[i].name;
        state_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &_arm_state.q(i));
        state_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &_arm_state.qd(i));
        state_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &_arm_state.tau(i));
    }
    if (with_gripper()) {
        const std::string jnt_name = joints()[6].name;
        state_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &_gripper_state.q);
        state_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &_gripper_state.qd);
        state_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &_gripper_state.tau);
    }
    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface>
HardwareInterface::export_command_interfaces() {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    std::vector<hardware_interface::CommandInterface> cmd_interfaces;
    cmd_interfaces.reserve(21);  // NOLINT: 7 joints * 3 cmd interfaces
    for (long i = 0; i < 6; ++i) {
        const std::string jnt_name = joints()[i].name;
        cmd_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &_arm_cmd.q(i));
        cmd_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &_arm_cmd.qd(i));
        cmd_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &_arm_cmd.tau(i));
    }
    if (with_gripper()) {
        const std::string jnt_name = joints()[6].name;
        cmd_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &_gripper_cmd.q);
        cmd_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &_gripper_cmd.qd);
        cmd_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &_gripper_cmd.tau);
    }
    return cmd_interfaces;
}

hardware_interface::return_type
HardwareInterface::
        read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    std::scoped_lock lk(state_mtx_);
    for (int i=0;i<6;++i) {
        _arm_state.q(i)   = state_buf_.q(i);
        _arm_state.qd(i)  = state_buf_.qd(i);
        _arm_state.tau(i) = state_buf_.tau(i);
    }
    if (with_gripper()) {
        _gripper_state.q   = state_buf_.g_q;
        _gripper_state.qd  = state_buf_.g_qd;
        _gripper_state.tau = state_buf_.g_tau;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::
        write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    std::scoped_lock lk(cmd_mtx_);
    for (int i=0;i<6;++i) {
        cmd_buf_.q(i)   = _arm_cmd.q(i);
        cmd_buf_.qd(i)  = _arm_cmd.qd(i);
        cmd_buf_.tau(i) = _arm_cmd.tau(i);
    }
    if (with_gripper()) {
        cmd_buf_.g_q   = _gripper_cmd.q;
        cmd_buf_.g_qd  = _gripper_cmd.qd;
        cmd_buf_.g_tau = _gripper_cmd.tau;
    }
    // Gains ya deberían haber sido fijados en perform_command_mode_switch().
    for (int i=0;i<7;++i) { cmd_buf_.kp[i]=_current_gains.kp[i]; cmd_buf_.kd[i]=_current_gains.kd[i]; }
    cmd_seq_.fetch_add(1, std::memory_order_release);


    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& /* stop_interfaces */
) {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    RCLCPP_INFO(get_logger(), "Switching control mode");
    for (const std::string& interface : start_interfaces) {
        const auto [name, type] = split_interface(interface);
        auto idx                = get_joint_id(name);

        if (type == HW_IF_POSITION) {
            RCLCPP_INFO(get_logger(), "Switching control mode POSITION");
            _current_gains.kp[idx] = _default_gains.kp[idx];
            _current_gains.kd[idx] = _default_gains.kd[idx];
        } else if (type == HW_IF_VELOCITY) {
            RCLCPP_INFO(get_logger(), "Switching control mode VELOCITY");
            _current_gains.kp[idx] = 0.0;
            _current_gains.kd[idx] = _default_gains.kd[idx];
        } else if (type == HW_IF_EFFORT) {
            RCLCPP_INFO(get_logger(), "Switching control mode EFFORT");
            _current_gains.kp[idx] = 0.0;
            _current_gains.kd[idx] = 0.0;
        } else {
            RCLCPP_ERROR(
                    get_logger(),
                    "Don't know how to configure interface '%s'",
                    interface.c_str()
            );
            return hardware_interface::return_type::ERROR;
        }
    }

    // clang-format off
    RCLCPP_INFO(get_logger(), "Updated proportional gains: %s", pretty_vector(_current_gains.kp).c_str());
    RCLCPP_INFO(get_logger(), "Updated derivative gains: %s", pretty_vector(_current_gains.kd).c_str());
    // clang-format on

    // _arm->lowcmd->setControlGain(_current_gains.kp, _current_gains.kd);
    // _arm->lowcmd->setGripperGain(_current_gains.kp[6], _current_gains.kd[6]);

    // // Set command to current state
    // _arm_cmd.q      = _arm_state.q;
    // _arm_cmd.qd     = _arm_state.qd;
    // _gripper_cmd.q  = _gripper_state.q;
    // _gripper_cmd.qd = _gripper_state.qd;


    // === Guardar cambios en el buffer compartido con write() ===
    {
        std::scoped_lock lk(cmd_mtx_);
        for (int i = 0; i < 6; ++i) {
            cmd_buf_.kp[i] = _current_gains.kp[i];
            cmd_buf_.kd[i] = _current_gains.kd[i];
            // Armar handover suave: partir del estado actual
            cmd_buf_.q(i)  = _arm_state.q(i);
            cmd_buf_.qd(i) = _arm_state.qd(i);
        }

        if (with_gripper()) {
            cmd_buf_.kp[6] = _current_gains.kp[6];
            cmd_buf_.kd[6] = _current_gains.kd[6];
            cmd_buf_.g_q   = _gripper_state.q;
            cmd_buf_.g_qd  = _gripper_state.qd;
        }
    }


    return hardware_interface::return_type::OK;
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//

void
HardwareInterface::saturate_torque() {
    const Vec6 original_tau = _arm_cmd.tau;
    _arm_cmd.tau = original_tau.cwiseMin(_arm_max_torque).cwiseMax(-_arm_max_torque);
    _gripper_cmd.tau =
            std::clamp(_gripper_cmd.tau, -_gripper_max_torque, _gripper_max_torque);

    if (original_tau != _arm_cmd.tau)
        RCLCPP_WARN(get_logger(), "Saturating input torque");
}

bool
HardwareInterface::with_gripper() const {
    std::string gripper_param = info_.hardware_parameters.at("gripper");
    to_lower_string(gripper_param);
    return gripper_param == "true";
}

long
HardwareInterface::get_joint_id(const std::string& joint_name) const {
    for (long i = 0; i < joints().size(); ++i) {
        if (joints()[i].name == joint_name) return i;
    }
    throw std::out_of_range(
            fmt::format("Unable to find joint '{}' with the joints of the robot")
    );
}

// ===========================================================================
// CAMBIO: loop de IO en hilo dedicado
void HardwareInterface::ioLoop() {
  while (io_running_.load()) {
    // 1. aplicar último comando
    {
      std::scoped_lock lk(cmd_mtx_);
      Vec6 tau_sat = cmd_buf_.tau.cwiseMin(_arm_max_torque).cwiseMax(-_arm_max_torque);
      double g_tau_sat = std::clamp(cmd_buf_.g_tau, -_gripper_max_torque, _gripper_max_torque);

      _arm->lowcmd->setControlGain(
        std::vector<double>(cmd_buf_.kp.begin(), cmd_buf_.kp.end()),
        std::vector<double>(cmd_buf_.kd.begin(), cmd_buf_.kd.end())
);

      if (with_gripper())
        _arm->lowcmd->setGripperGain(cmd_buf_.kp[6], cmd_buf_.kd[6]);

      _arm->setArmCmd(cmd_buf_.q, cmd_buf_.qd, tau_sat);
      if (with_gripper())
        _arm->setGripperCmd(cmd_buf_.g_q, cmd_buf_.g_qd, g_tau_sat);
    }

    // 2. copiar estado
    SharedState tmp;
    for (int i=0;i<6;++i) {
      tmp.q(i)   = _arm->lowstate->q[i];
      tmp.qd(i)  = _arm->lowstate->dq[i];
      tmp.tau(i) = _arm->lowstate->tau[i];
    }
    if (with_gripper()) {
      tmp.g_q   = _arm->lowstate->q[6];
      tmp.g_qd  = _arm->lowstate->dq[6];
      tmp.g_tau = _arm->lowstate->tau[6];
    }
    {
      std::scoped_lock lk(state_mtx_);
      state_buf_ = tmp;
    }
  }
}

// CAMBIO: utilidades para parar hilo
void HardwareInterface::stopIoThread() {
  bool expected = true;
  if (io_running_.compare_exchange_strong(expected,false)) {
    if (io_thread_.joinable()) io_thread_.join();
  }
}

// ===========================================================================

//  ____  _        _   _
// / ___|| |_ __ _| |_(_) ___ ___
// \___ \| __/ _` | __| |/ __/ __|
//  ___) | || (_| | |_| | (__\__ \
// |____/ \__\__,_|\__|_|\___|___/
//

/**
 * @bried Convert in-place a string to lower case.
 *
 * @param[in,out] str       The string to be converted.
 */
static void
to_lower_string(std::string& str) {
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {
        return std::tolower(c);
    });
}

static std::pair<std::string, std::string>
split_interface(const std::string& in) {
    const auto sep_id = in.find('/');
    return std::make_pair(
            in.substr(0, sep_id), in.substr(sep_id + 1, in.size() - sep_id - 1)
    );
}

//  _____                       _
// | ____|_  ___ __   ___  _ __| |_
// |  _| \ \/ / '_ \ / _ \| '__| __|
// | |___ >  <| |_) | (_) | |  | |_
// |_____/_/\_\ .__/ \___/|_|   \__|
//            |_|
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
        unitree::z1::HardwareInterface, hardware_interface::SystemInterface
);