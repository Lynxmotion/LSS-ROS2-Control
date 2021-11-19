//
// Created by guru on 12/27/19.
//

#pragma once

//#include "rclcpp/client.hpp"
//#include "rclcpp/rate.hpp"

#include <memory>
#include <string>
#include <deque>

/******* Ros2 Controls and related includes ****/
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>


/******* joint calibration interface *******/
//#include <humanoid_model_msgs/msg/joint_calibration.hpp>

/******* Lynxmotion library *******/
#include <DeviceArray.h>
#include <LssBus.h>
#include <rclcpp/macros.hpp>

#include "ros2_lss_bus//visibility_control.h"


namespace lynxmotion {

    constexpr const auto HW_IF_STIFFNESS = "stiffness";
    constexpr const auto HW_IF_CURRENT = "current";

    class LssBusHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
    public:
        using return_type = hardware_interface::return_type;

        RCLCPP_SHARED_PTR_DEFINITIONS(LssBusHardware)

        typedef enum {
            Unconfigured,
            Stopped,
            Idle,
            ReadingStates,
            WritingCommands
        } BusState;

        LssBusHardware();

        ~LssBusHardware() override;

        /* TODO to get to parity with old LSS joint controller:
         *   PARAMETERS   required => offsets, inverts
         *                limp => will need to make as a command such as mode, or possibly effort==0
         *                deprecated? => gains.gravity, gravity-bias, compliance-params (stiffness, max-amps, ...)
         *                I think compliance will be a higher-order controller now
         *   INTERFACES
         *                JointCalibration => origin, range (per joint)
         *                CompliantJointParams => gravity, torque-{limit, bias, active, limp}
         *
         *   BEHAVIOUR
         *                In calibration mode, offsets/etc are sent to servos instead of position commands.
         */


        LSS_HARDWARE_PUBLIC
        return_type configure(const hardware_interface::HardwareInfo & info) override;

        LSS_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        LSS_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        LSS_HARDWARE_PUBLIC
        return_type start() override;

        LSS_HARDWARE_PUBLIC
        return_type stop() override;

        LSS_HARDWARE_PUBLIC
        return_type read() override;

        LSS_HARDWARE_PUBLIC
        return_type write() override;

    protected:
        class StateData {
        public:
            std::vector<double> position;
            std::vector<double> velocity;
            std::vector<double> effort;
            std::vector<double> current;    // as in amps
        };

        class CommandData {
        public:
            std::vector<double> effort;
            std::vector<double> stiffness;
        };

        class CommandInterfaceConfig {
        public:
          inline CommandInterfaceConfig()
          : min(-1.0), max(1.0)
          {}

          double min, max;
        };
        class JointConfig {
        public:
          inline JointConfig()
              : bus_id(-1), invert(false)
          {}

          std::string name;
          short bus_id;

          // joint parameters
          bool invert;

          CommandInterfaceConfig position;
          CommandInterfaceConfig velocity;
        };

        // this flag is controlled by the start/stop methods
        bool active;

        // required parameters
        std::string port;

        // optional parameters
        std::string hw_name;
        std::string joint_prefix;   // optional prefix to joint names before the joint's bus ID should be matched
        long long baudrate;         // port baudrate
        double Tk;                  // torque coefficient for converting amps to Torque effort  (defaults to LSS HT1)
                                    // good formulas at https://www.motioncontroltips.com/faq-whats-relationship-voltage-dc-motor-output-speed
        double quiescentCurrent;    // current in milli-amps always required by each servo even when idle or limp (current used by microprocessor, etc)
        bool log_bus_warnings;      // true if bus warnings should be logged (default is not logged)
        bool log_bus_errors;        // true if bus errors should be logged (default is logged)


        // joints
        std::vector<JointConfig> hw_joints;
        //std::vector<short> hw_joint_bus_id;
        lss::DeviceIndex hw_joint_index;
        lss::DeviceIndex hw_joint_index_inverted;   // the inverse of
                                                    // hw_joint_index (cached)
        //std::vector<unsigned long> hw_joint_flags;

        StateData state_;

        // Position is always sent and we can optimize it by directly binding
        // our position command to Request collection
        std::vector<double> command_position_;
        std::vector<lss::Request> command_position_lss_;

        // other commands are only sent if change is detected
        CommandData command_;

        // LSS bus hardware interface
        lss::Bus bus;
        std::vector<lss::Request> state_request;
        std::vector<lss::Request> state_reply;
        bool reply_pending;

    protected:
        //void publish_diagnostics();

//        rclcpp::GenericRate<std::chrono::system_clock> connection_rate_;

        short extract_bus_id(const std::string& joint_name);
        inline bool is_inverted(size_t jointOrdinal) const
            { return hw_joints[jointOrdinal].invert; }

    protected:
        size_t remaining;
        //std::mutex remaining_mutex;

        size_t overruns;
        size_t success, failed, consecutive_failures;

        BusState bus_state; // current loop task/state

//        void calibrate(humanoid_model_msgs::msg::JointCalibration::SharedPtr msg);

        //inline rclcpp::Logger get_logger() { return rclcpp::get_logger("lss_hardware"); }
    };


} // ns:lynxmotion
