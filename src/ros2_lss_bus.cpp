//
// Created by guru on 12/27/19.
//

#include "ros2_lss_bus/ros2_lss_bus.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include <rcutils/logging_macros.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
        lynxmotion::LssBusHardware,
        hardware_interface::SystemInterface
)

namespace lynxmotion {

    static const char* lss_hardware_logger = "lss_bus";

    constexpr const double ssc32_usecs_per_revolution = 4000;
    constexpr double lss_usec_to_radians(int uSecs) {
        return (double)uSecs * M_PI_2 / ssc32_usecs_per_revolution;
    }
    constexpr int radians_to_lss_usec(double radians) {
        return (int)(radians * ssc32_usecs_per_revolution / M_PI_2);
    }

    constexpr double lss_degrees_to_radians(int lss_degrees) {
        return (double)lss_degrees * M_PI / 1800.0;
    }
    constexpr int radians_to_lss_degrees(double radians) {
        return (int)(radians * 1800.0 / M_PI);
    }

    constexpr double lss_micros_to_diagnostic_time(unsigned long micros) {
        return (double)micros / 1000.0;
    }

    template<class T>
    constexpr T clamp(T x, T min, T max) {
        return (x < min)
            ? min
            : (x > max)
                ? max
                : x;
    }

#if 0
    typedef enum {
        Stopped,
        Starting,
        Stopping,
        Idle,
        Querying,
        Commanding
    } hw_process_state_e;

    // single thread manages all lss hardware I/O
    pthread_t lss_hw_process;
    std::mutex iops_mutex;
    hw_process_state_e hw_process_state;
    std::vector<LynxmotionServoHardware*> iops;

    void* lss_hw_process_thread(void*);
    bool start_hardware_iop(LynxmotionServoHardware* p);
    void stop_hardware_iop(LynxmotionServoHardware* p);
#endif

    LssBusHardware::LssBusHardware()
        : active(false), baudrate(115200), Tk(0), quiescentCurrent(70), log_bus_warnings(false), log_bus_errors(true),
          reply_pending(false), remaining(0), overruns(0), success(0), failed(0), consecutive_failures(0), bus_state(Unconfigured)
    {
    }

    LssBusHardware::~LssBusHardware() {
        bus.close();
    }

    std::string get_parameter(const std::string& name, std::string default_value, const std::unordered_map<std::string, std::string>& params) {
        auto i = params.find(name);
        if(i != params.end()) {
            return i->second;
        } else
            return default_value;
    }

    double get_parameter(const std::string& name, double default_value, const std::unordered_map<std::string, std::string>& params) {
        auto i = params.find(name);
        if(i != params.end()) {
            std::string msg;
            if(i->second.empty())
                throw std::runtime_error("parameter " + name + " is blank");

            char *ep;
            const char *p = i->second.c_str();
            double v = strtod(p, &ep);
            if(*ep != 0)
                throw std::runtime_error("parameter " + name + " is not a number");
            return v;
        } else
            return default_value;
    }

#if 0
    bool get_parameter(const std::string& name, bool default_value, const std::unordered_map<std::string, std::string>& params) {
        auto i = params.find(name);
        if(i != params.end()) {
            std::string msg;
            if(i->second.empty())
                return true;    // if parameter is present but blank, assume true

            const char* v = i->second.c_str();
            if(strcasecmp(v, "true") ==0)
                return true;
            else if(strcasecmp(v, "false") ==0)
                return false;
            else
                throw std::runtime_error("expecting parameter " + name + " to be true or false");
        } else
            return default_value;
    }
#endif

    long long get_parameter(const std::string& name, long long default_value, long long min_value, long long max_value,  const std::unordered_map<std::string, std::string>& params) {
        auto i = params.find(name);
        if(i != params.end()) {
            std::string msg;
            if(i->second.empty())
                throw std::runtime_error("parameter " + name + " is blank");

            char *ep;
            const char *p = i->second.c_str();
            long long v = strtoll(p, &ep, 10);
            if(*ep != 0) {
                throw std::runtime_error("parameter " + name + " is not a number");
            } else if(v < min_value) {
                std::ostringstream s;
                s << "parameter " << name << " must be greater than " << min_value;
                throw std::runtime_error(s.str());
            } else if(v > max_value) {
                std::ostringstream s;
                s << "parameter " << name << " must be less than " << max_value;
                throw std::runtime_error(s.str());
            }
            return v;
        } else
            return default_value;
    }

    std::string get_required_parameter(const std::string& name, const std::unordered_map<std::string, std::string>& params) {
        auto i = params.find(name);
        if(i != params.end()) {
            return i->second;
        } else {
            std::string msg = "missing required parameter " + name;
            throw std::runtime_error(msg);
        }
    }

    short LssBusHardware::extract_bus_id(const std::string& joint_name)
    {
        const char* p = joint_name.c_str();
        if(!joint_prefix.empty()) {
            // the joint name must contain the prefix or we fail (skip)
            if(strncmp(p, joint_prefix.c_str(), joint_prefix.length()) !=0) {
                return -1;
            } else
                p += joint_prefix.length();
        } else if(!isdigit(*p)) {
                RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "joint %s has non-numeric prefix but no prefix parameter set, skipping.", joint_name.c_str());
                return -1;  // garbage at beginning of joint ID
        }


        // we should now only have a numeric ID
        char* ep;
        long id = strtol(p, &ep, 10);
        if(*ep != 0) {
            RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "unsupported non-numeric characters at end of LSS joint name %s", joint_name.c_str());
            return -1;  // garbage at end of joint ID
        } else if(id >= lss::BroadcastID) {
            RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "joint %s specifies a LSS bus IDs out of range, use 1 to 254", joint_name.c_str());
            return -1;  // garbage at end of joint ID
        }
        return id;
    }

#if defined(ROS_FOXY)
    LssBusHardware::return_type
    LssBusHardware::configure(const hardware_interface::HardwareInfo & info) {
      //connection_rate_.sleep();
      if (configure_default(info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
      }
#else
  LssBusHardware::return_type
  LssBusHardware::on_init(const hardware_interface::HardwareInfo &info) {
        //connection_rate_.sleep();
        if (hardware_interface::SystemInterface::on_init(info) !=
                return_type::SUCCESS) {
          return return_type::ERROR;
        }
#endif

        try {
            // load required parameters
            port = get_required_parameter("port", info.hardware_parameters);

            const char* port_basename = strrchr(port.c_str(), '/');

            // load optional parameters
            hw_name = get_parameter("name", port_basename ? port_basename + 1 : port.c_str(), info.hardware_parameters);
            joint_prefix = get_parameter("prefix", "", info.hardware_parameters);
            baudrate = get_parameter("baudrate", 230400LL, 300, 7372800, info.hardware_parameters);
            Tk = get_parameter("Tk", 0.003625, info.hardware_parameters);
            quiescentCurrent = get_parameter("quiescentCurrent", 70.0, info.hardware_parameters);

            std::string logBus = get_parameter("log", "errors", info.hardware_parameters);
            if(strcasecmp(logBus.c_str(), "errors") ==0) {
                log_bus_warnings = false;
                log_bus_errors = true;
            } else if(strcasecmp(logBus.c_str(), "warnings") ==0) {
                log_bus_warnings = false;
                log_bus_errors = true;
            } else if(strcasecmp(logBus.c_str(), "all") ==0) {
                log_bus_warnings = true;
                log_bus_errors = true;
            }
        } catch (const std::runtime_error& e) {
            RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "configure error: %s", e.what());
            return return_type::ERROR;
        }

        RCUTILS_LOG_INFO_NAMED(lss_hardware_logger, "configuring LSS bus %s "
                                                    "for port %s @ %lld baud",
                               hw_name.c_str(), port.c_str(), baudrate);

        // allocate status
        size_t jc = info.joints.size();
        state_.position.resize(jc);
        state_.effort.resize(jc);
        state_.velocity.resize(jc);
        state_.current.resize(jc);

        command_position_.resize(jc);
        command_.effort.resize(jc);
        command_.stiffness.resize(jc, -3.0);

        // build the query transaction
        std::vector<lss::Request> queries;
        std::vector<lss::Request> commands;
        for (auto j: info.joints) {
          // add a new joint to the collection and get the reference
          hw_joints.emplace_back();
          auto& jcfg = hw_joints.back();

          jcfg.name = j.name;

          // load parameters from the joints node
          for(const auto& p: j.parameters) {
            if(p.first == "id") {
              // receiving the LSS bus ID from a parameter
              jcfg.bus_id = ::strtod(p.second.c_str(), nullptr);
            } else if(p.first == "invert") {
              jcfg.invert = !p.second.empty()
                  && (toupper(p.second[0])=='T' || p.second=="1");
            }
          }

          if(jcfg.bus_id <0)
            jcfg.bus_id = extract_bus_id(j.name);

          // load command interface config from the command_interface nodes
          for(const auto& iface: j.command_interfaces) {
            CommandInterfaceConfig* ci;
            if(iface.name == "position")
              ci = &jcfg.position;
            else if(iface.name == "velocity")
              ci = &jcfg.velocity;
            else
              ci = nullptr;
            if(ci) {
              if(!iface.min.empty())
                ci->min = ::strtod(iface.min.c_str(), nullptr);
              if(!iface.max.empty())
                ci->max = ::strtod(iface.max.c_str(), nullptr);
             }
          }

          // todo: how are joints named in config?
          RCUTILS_LOG_INFO_NAMED(lss_hardware_logger,
               "  Joint %s  id:%d min:%4.3f max:%4.3f",
                 jcfg.name.c_str(), jcfg.bus_id,
                 jcfg.position.min, jcfg.position.max);

          // build our command and state LSS bus requests
          command_position_lss_.emplace_back(jcfg.bus_id, lss::command::D, 0);

          // query commands
          queries.emplace_back(jcfg.bus_id, lss::command::QD);
          queries.emplace_back(jcfg.bus_id, lss::command::QS);
          queries.emplace_back(jcfg.bus_id, lss::command::QC);

          // transmitting commands
          commands.emplace_back(jcfg.bus_id, lss::command::D);
          //commands.emplace_back(jcfg.bus_id, Lss::Command::MMD);
          //commands.emplace_back(jcfg.bus_id, Lss::Command::L);
          // Angular Holding Stifness
          // Angular Stiffness
        }

        // todo: query bus for servo information?
        // todo: load offsets? - maybe just provide a way to set offsets within the servo (less confusing)

        bus_state = Stopped;
        RCUTILS_LOG_INFO_NAMED(lss_hardware_logger, "configuring of LSS bus %s complete", hw_name.c_str());
#if defined(ROS_FOXY)
        return return_type::OK;
#else
        return return_type::SUCCESS;
#endif
  }


    std::vector<hardware_interface::StateInterface>
    LssBusHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(
                    hardware_interface::StateInterface(
                            info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION,
                            &state_.position[i]));
            state_interfaces.emplace_back(
                    hardware_interface::StateInterface(
                            info_.joints[i].name,
                            hardware_interface::HW_IF_VELOCITY,
                            &state_.velocity[i]));
            state_interfaces.emplace_back(
                    hardware_interface::StateInterface(
                            info_.joints[i].name,
                            hardware_interface::HW_IF_EFFORT,
                            &state_.effort[i]));
            state_interfaces.emplace_back(
                    hardware_interface::StateInterface(
                            info_.joints[i].name, HW_IF_CURRENT,
                            &state_.current[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    LssBusHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(
                    hardware_interface::CommandInterface(
                            info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION,
                            &command_position_[i]));
            command_interfaces.emplace_back(
                    hardware_interface::CommandInterface(
                            info_.joints[i].name,
                            hardware_interface::HW_IF_EFFORT,
                            &command_.effort[i]));
            command_interfaces.emplace_back(
                    hardware_interface::CommandInterface(
                            info_.joints[i].name,
                            HW_IF_STIFFNESS,
                            &command_.stiffness[i]));
        }

        return command_interfaces;
    }

#if defined(ROS_FOXY)
    LssBusHardware::return_type LssBusHardware::start()
#else
LssBusHardware::return_type LssBusHardware::on_activate(
        const rclcpp_lifecycle::State &
  )
#endif
    {
        // open the LSS bus port
        lss::platform::ChannelDriverError bus_err
            = bus.open(port.c_str(), baudrate);
        if(bus_err != lss::platform::DriverSuccess) {
            std::string msg;
            RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "cannot open port %s: %s", port.c_str(), lss_bus_driver_error_str(bus_err));
            return return_type::ERROR;
        }

        // send initial servo state as a broadcast
        //bus.transmit(LynxPacket(254, LssLimp | LssAction));
        bus.write({lss::Request(254, lss::command::EM, 0),
                   lss::Request(254, lss::command::FPC,3),
                   lss::Request(254, lss::command::LED, lss::Red),
                   lss::Request(254, lss::command::AS,-3),
                   lss::Request(254, lss::command::AH, -3),
                   lss::Request(254, lss::command::SD, 700)
        });


        // create our compliant joints
        // todo: probably move this to an on_configure()
        //joints.clear();
        //for(size_t j=0; j<config.joints.size(); j++) {
        //    joints.push_back(std::make_shared<CompliantJoint>(config.joints[j], config.names[j].c_str()));
        //}
        // change color on servos on joints we control
        hw_joint_index.clear();
        state_request.clear();
        state_reply.clear();
        for(auto& j: hw_joints) {
            hw_joint_index.append(j.bus_id);

            // set the LED
            bus.write(lss::Request(j.bus_id, lss::command::LED, lss::Blue));

            // build the state reply message for all joints
            state_reply.emplace_back(j.bus_id, lss::command::QD);
            state_reply.emplace_back(j.bus_id, lss::command::QC);
            state_reply.emplace_back(j.bus_id, lss::command::QS);
        }
        hw_joint_index_inverted = hw_joint_index.invert();

        // use broadcast slots to query for state
        bus.write_slot_config(hw_joint_index);
        state_request.emplace_back(lss::BroadcastID, lss::command::QD);
        state_request.emplace_back(lss::BroadcastID, lss::command::QC);
        state_request.emplace_back(lss::BroadcastID, lss::command::QS);

        assert(command_position_.size() == command_position_lss_.size());

#if defined(ROS_FOXY)
        return return_type::OK;
#else
        return return_type::SUCCESS;
#endif
    }


#if defined(ROS_FOXY)
      LssBusHardware::return_type LssBusHardware::stop()
#else
LssBusHardware::return_type LssBusHardware::on_deactivate(
    const rclcpp_lifecycle::State &
    )
#endif
    {
        // turn servos red
        for(auto& j: hw_joints) {
          bus.write(lss::Request(j.bus_id, lss::command::LED, lss::Red));
        }

        bus.close();

#if defined(ROS_FOXY)
        return return_type::OK;
#else
        return return_type::SUCCESS;
#endif
    }

    hardware_interface::return_type LssBusHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
      int n;
        if(reply_pending) {
          if(0 < (n = bus.update(state_reply.begin(), state_reply.end()))) {
            auto preq = state_reply.begin();
            while(n-- > 0) {
              // If one or more servos didn't reply then some requests
              // in our state_reply may not have been parsed, we'll just
              // catch'em the next round.
              if(preq->flags.parsed) {
                // resolve the joint ID to our joint array index
                size_t idx = hw_joint_index_inverted[preq->id];

                switch (preq->command) {
                  case lss::command::QD:
                    state_.position[idx] = lss_degrees_to_radians(preq->args[0]);
                    break;
                case lss::command::QC: {
                    auto milliamps = (double)preq->args[0];
                    state_.current[idx] = milliamps;
                    auto v = (milliamps > quiescentCurrent)
                        ? milliamps - quiescentCurrent
                        : 0.0;
                    state_.effort[idx] = Tk * v;
                  } break;
                case lss::command::QS:
                  state_.velocity[idx] = lss_usec_to_radians(preq->args[0]);
                    break;
                default: break;// nothing
                }
              }
              preq++;
            }
          }
        }

        // request state for next tick
        if(bus.write(state_request.begin(), state_request.end()) >0)
          reply_pending = true;

        // read values from the bus's state realtime buffer to state handles
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type LssBusHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
      // convert positions from Ros to Lss
      for(size_t i=0; i < command_position_.size(); i++) {
        auto& dev = command_position_lss_[i];
        bool enabled = command_.effort[i] > 0.0;
        if(enabled) {
          // send a position update
          const auto& joint_cfg = hw_joints[i];

          // ensure position is within limits
          auto position = command_position_[i];
          if(position < joint_cfg.position.min)
            position = joint_cfg.position.min;
          else if(position > joint_cfg.position.max)
            position = joint_cfg.position.max;

          dev.command = lss::command::D;
          dev.nargs = 1;
          dev.args[0] = radians_to_lss_degrees(position);
        } else {
          // send the limp command
          dev.command = lss::command::L;
          dev.nargs = 0;
        }
      }

      // send the positions to the bus
      bus.write(command_position_lss_);

        // write values from the command buffer to the command realtime buffer
        // non-realtime write access
        return hardware_interface::return_type::OK;
    }

#if 0
    void LssBusHardware::calibrate(humanoid_model_msgs::msg::JointCalibration::SharedPtr) {
        // calibration data
        std::vector<short> jointOrdinals;
        std::vector<double> newOriginDegrees;
        bool commit = msg->commit;

        RCUTILS_LOG_INFO_NAMED(lss_hardware_logger, "calibrating %s", hw_name.c_str());

        /*
         * Queue calibration information from servos
         */
        std::vector<LynxPacket> packets;
        std::vector<LynxPacket> queries;
        size_t nj = 0;
        // search the joints on our bus for each joint specified in the message
        // any unmatched joints get ignored (assumed to be part of another bus)
        for (size_t jn = 0, _jn = hw_joints.size(); jn < _jn; jn++) {
            auto &j = hw_joints[jn];
            auto bus_id = hw_joint_bus_id[jn];

            // search the calibration message for this joint
            auto j_itr = std::find(msg->joints.begin(), msg->joints.end(), j);
            if (j_itr != msg->joints.end()) {
                // joint calibration entry find, add to our calibration transaction
                auto ordinal = j_itr - msg->joints.begin();         // get the offset into the joints array
                auto deg = msg->origin[ordinal] * 1800 / M_PI;      // covert the ros2 radian position into degrees
                if (is_inverted(nj))
                    deg *= -1;

                // save joint calibration
                jointOrdinals.emplace_back(jn);
                newOriginDegrees.emplace_back(deg);      // convert from radians to tenths of degrees

                // disable FPC and go Limp on joints
                packets.emplace_back(bus_id, LssAction | LssFilterPoleCount, 0);
                packets.emplace_back(bus_id, LssAction | LssLimp);

                // queries will be collected until the end
                queries.emplace_back(bus_id, LssQuery | LssPosition | LssDegrees);
                queries.emplace_back(bus_id, LssQuery | LssOriginOffset);
            }

            nj++;
        }

        // the the queries to the end of packets
        packets.insert(packets.end(), queries.begin(), queries.end());
        queries.clear();

#if 0
        // actions sent, now do queries
        for (size_t i = 0, _i = joints.size(); i < _i; i++) {
            auto jn = joints[i];
            auto bus_id = hw_joint_bus_id[jn];
            packets.emplace_back(bus_id, LssQuery | LssPosition | LssDegrees);
            packets.emplace_back(bus_id, LssQuery | LssOriginOffset);
        }
#endif

        // build our transaction
        if (!packets.empty()) {
            auto qtx = prepare(packets.begin(), packets.end(), 35000);
            qtx.then([this, jointOrdinals, newOriginDegrees, commit](const LssTransaction& rx) {
                std::stringstream ss;
                ss << (commit ? "committing " : "") << "calibrating " << std::setprecision(4) << std::endl;

                RCUTILS_LOG_INFO_NAMED(lss_hardware_logger, "   completing calibration for %s",
                                       hw_name.c_str());

                std::vector<LynxPacket> packets;
                auto p = rx.packets().begin(), _p = rx.packets().end();

                // skip actions
                while (p->command & LssAction)
                    p++;

                // process query responses
                size_t jointCalibOrdinal = 0;        // index for our cached origin values
                while (p != _p) {

                    // expect servo position
                    if (p->matches(LssQuery | LssPosition)) {
                        if (p->hasValue) {
                            auto measuredPosition = p->value;       // current servo position
                            //p->id = 0;                            // todo: remove this, const, we build new packet collection (disable this packet for next transmission)
                            p++;                                    // we should now be at a LssOriginOffset packet

                            if(jointCalibOrdinal >= jointOrdinals.size())
                                throw std::runtime_error("calibration reply doesnt match our query");

                            auto jn = jointOrdinals[jointCalibOrdinal];
                            if(hw_joint_bus_id[jn] != p->id)
                                throw std::runtime_error("calibration reply for packet doesnt match our query");

                            // expect servo origin offset and compute new offset
                            if (p != packets.end() && p->matches(LssQuery | LssOriginOffset) && p->hasValue) {
                                auto currentOrigin = p->value;            // current offset
                                auto absoluteMeasuredPosition = currentOrigin + measuredPosition;
                                while (absoluteMeasuredPosition > 1800)
                                    absoluteMeasuredPosition -= 3600;
                                while (absoluteMeasuredPosition < -1800)
                                    absoluteMeasuredPosition += 3600;

                                auto calibratedPosition = newOriginDegrees[jointCalibOrdinal];               // desired servo position after offset is moved
                                while (calibratedPosition > 1800)
                                    calibratedPosition -= 3600;
                                while (calibratedPosition < -1800)
                                    calibratedPosition += 3600;

                                auto calibratedOrigin =
                                        absoluteMeasuredPosition - calibratedPosition;                 // desired factory offset

                                // update our local joint
                                //auto joint = joints[jointOrdinal];

                                auto rads = (double) calibratedPosition * M_PI / 1800.0;
                                state_.position[jn] = rads;
                                command_.position[jn] = rads;

                                // convert the origin query to an origin set command
                                packets.emplace_back(
                                        p->id,
                                        (commit ? LssConfig : LssAction) | LssOriginOffset,
                                        calibratedOrigin);
                                //packets.emplace_back(p->id, LssAction | LssPosition | LssDegrees, dofs);
                                //packets.emplace_back(p->id, LssFilterPoleCount, joint->cpr);
                                //p->command = LssAction | LssOriginOffset;
                                //p->set(fofs);

                                bool update = calibratedOrigin != currentOrigin;
                                ss << "      " << hw_joints[jn] << "|"
                                   << measuredPosition << "°" << currentOrigin << "=>"
                                   << calibratedPosition << "°" << calibratedOrigin << (update ? '*' : ' ')
                                   << std::endl;

                                p++;
                            } else
                                throw std::runtime_error("expected servo origin offset reply in calibration message");

                            jointCalibOrdinal++;
                        } else
                            throw std::runtime_error("servo responded to position request without a value");
                    } else
                        throw std::runtime_error("expected servo position reply in calibration message");

                }

                // output calibration data
                std::cout << ss.str() << std::endl;

                // send calibration update transaction
                // no need for a transaction since they are all actions
                bus.send(packets.begin(), packets.end());
            });

            // place the prepared transaction into the queue
            queue(qtx);
        }
    }
#endif

#if 0
    unsigned long LynxmotionServoHardware::read_state(RealtimeArgs& args) {
        // any side-band queue to send?
        if(!queue_.empty()) {
            auto qtx = queue_.front();
            queue_.pop_front();
            bus.send(qtx);
        }


        // todo: RealtimeArgs should be copy/move since we expect these methods to be asynchronous and
        //       therefor using references arent safe (unless RealtimeArgs is kept in object mem)
        if (queryTx) {
            size_t packets_per_joint = queryTx->packets().size() / hw_joints.size();
            bus.send(queryTx)
                    .then([this, args, packets_per_joint](const LssTransaction& rx) {
                        //auto ustart = micros();
                        auto& packets = rx.packets();
                        args.diagnostic( Diagnostic(
                                lss_micros_to_diagnostic_time(rx.ttfr),
                                lss_micros_to_diagnostic_time(rx.ttc),
                                1,
                                0));

                        int n = 0;
                        for (auto &p : packets) {
                            size_t jn = n / packets_per_joint;
                            bool invert = is_inverted(jn);
                            if (p.command & LssPosition) {
                                auto value = lss_degrees_to_radians(p.value);
                                state_.position[jn] = invert ? -value : value;
                            } else if(p.command & LssSpeed) {
                                auto value = lss_usec_to_radians(p.value);
                                state_.velocity[jn] = invert ? -value : value;
                            } else if (p.command & LssCurrent) {
                                state_.current[jn] = p.value;
                                auto v = (p.value > quiescentCurrent)
                                         ? p.value - quiescentCurrent
                                         : 0;
                                state_.effort[jn] = Tk * v;
                            }
                            n++;
                        }

                        // now that reads are done, write commands
                        // todo: we should just signal lss_realtime that writing is ready
                        n = 0;
                        size_t cmd_packets_per_joint = commandTx->packets().size() / hw_joints.size();
                        for (auto &p : commandTx->packets()) {
                            size_t jn = n / cmd_packets_per_joint;
                            bool invert = is_inverted(jn);

                            if(command_.effort[jn] <= 0.0) {
                                // servo is limp
                                p.enable(p.command & LssLimp);
                            } else {
                                // servo is active
                                if (p.command & LssPosition) {
                                    auto value = radians_to_lss_degrees(command_.position[jn]);
                                    p.set(invert ? -value : value);
                                } else if (p.command & LssMaxDuty) {
                                    int v = (int) clamp(
                                            quiescentCurrent + command_.effort[jn] / Tk,
                                            0.0, 1024.0);
                                    p.set(v);
                                } else if (p.command & LssLimp) {
                                    // if effort is zero or less, then make the servo go limp
                                    p.enable(false);
                                }
                            }
                            n++;
                        }
                        bus.send(commandTx);
#if 0
                                // todo: get rid of this callback after testing
                                .then([this, packets_per_joint](const LssTransaction&) {
                                    int x =54;
                                });
#endif
                    })
                    .otherwise([args](const LssTransaction& rx) {
                        //printf("transaction timeout\n");
                        args.diagnostic( Diagnostic(
                                lss_micros_to_diagnostic_time(rx.ttfr),
                                lss_micros_to_diagnostic_time(rx.ttc),
                                0,
                                1
                                ));
                    });
                    // channel->queryComplete(tx, *_joint_state_msg, channel->joint_state_msg_offset);
        }
        return 0;
    }

    unsigned long LynxmotionServoHardware::write_command(RealtimeArgs&) {
        // disabled in realtime class
        // we are writing commands within the read_state function above
        return 0;
    }
#endif

#if 0
    void LynxmotionServoHardware::spin() {
        try {
            //RCLCPP_DEBUG(get_logger(), "Querying for current IMU data");

            if (remaining > 0) {
                // report an overrun
                overruns++;
                return;
            }

            if (active) {
#if 0   // todo: we probably want to provide a testing mode again (will need to convert this to a node)
                if (_test_joint_state_msg && get_parameter("testpose").get_value<bool>()) {
                    _test_joint_state_msg->header.stamp = stamp;
                    _joint_state_pub->publish(*_test_joint_state_msg);
                    return;
                }
#endif

                if (!channel->ready() || channel->joints.empty() || !channel->queryTx)
                    continue;


                remaining_mutex.lock();
                remaining++;
                remaining_mutex.unlock();

                if (channel->calibrate) {
                    auto calibrate = channel->calibrate;
                    std::shared_ptr<LssTransaction> queryTx;

                    try {
                        queryTx = calibrate->query();
                    } catch (const std::exception &e) {
                        if (calibrate->attempts > 5) {
                            RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "calibration failed multiple times, aborting: %s", e.what());
                            channel->calibrate.reset();
                        } else
                            RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "calibration failed: %s", e.what());
                    }

                    channel->send(queryTx)
                            .then([this, calibrate, channel](const LssTransaction &tx) {
                                std::shared_ptr<LssTransaction> updateTx;
                                try {
                                    updateTx = calibrate->update(tx);
                                } catch (const std::exception &e) {
                                    if (calibrate->attempts > 5) {
                                        RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "calibration failed multiple times, aborting: %s",
                                                                e.what());
                                        channel->calibrate.reset();
                                    } else
                                        RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "calibration failed: %s", e.what());
                                }

                                if (updateTx) {
                                    channel->send(updateTx)
                                            .then([this, channel](LssTransaction &) {
                                                remaining_mutex.lock();
                                                remaining--;
                                                remaining_mutex.unlock();

                                                // success, remove the request
                                                channel->calibrate.reset();
                                            })
                                            .otherwise([this, channel](LssTransaction &) {
                                                RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "Failed to transmit servo calibration on %s",
                                                                        channel->config.name.c_str());
                                                remaining_mutex.lock();
                                                remaining--;
                                                remaining_mutex.unlock();
                                            });
                                } else {
                                    // success when no update TX given
                                    remaining_mutex.lock();
                                    remaining--;
                                    remaining_mutex.unlock();

                                    // success, remove the request
                                    channel->calibrate.reset();
                                }
                            })
                            .otherwise([this, channel](LssTransaction &) {
                                RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "Failed to query servo calibration on %s",
                                                        channel->config.name.c_str());
                                remaining_mutex.lock();
                                remaining--;
                                remaining_mutex.unlock();
                            });
                    continue;
                }

                // default to normal servo update loop
                auto qstart = micros();
                channel->send(channel->queryTx)
                        .then([this, qstart, &channel](const LssTransaction &tx) {
                            auto ustart = micros();

                            channel->queryComplete(tx, *_joint_state_msg, channel->joint_state_msg_offset);

                            std::vector<LynxPacket> updates;
                            for (auto &j: channel->joints) {
                                if (!j->isEnabled())
                                    continue;
                                j->state = CompliantJoint::ComplianceLimp;

                                // set color of servo to indicate joint state (Compliance, Limp or Holding/Moving)
                                switch (j->state) {
                                    case CompliantJoint::NegativeCompliance:
                                    case CompliantJoint::PositiveCompliance:
                                        updates.emplace_back(j->joint, LssLEDColor, LssMagenta);
                                        break;

                                    case CompliantJoint::ComplianceLimp:
                                        updates.emplace_back(j->joint, LssLEDColor, LssRed);
                                        break;

                                    default:
                                        updates.emplace_back(j->joint, LssLEDColor, LssBlue);
                                        break;
                                }

                                if (j->cpr_changed) {
                                    //if(j.cpr ==0) {
                                    //    updates.emplace_back(j.joint, LssMotionControl, 1);
                                    //    updates.emplace_back(j.joint, LssMotionControl, 0);
                                    //} else
                                    updates.emplace_back(j->joint, LssFilterPoleCount, j->cpr);
                                    updates.emplace_back(j->joint, LssPosition | LssAction | LssDegrees,
                                                         j->position.target());
                                    j->cpr_changed = false;
                                    // fill the CPR
                                    //for(int i=1; i<j.cpr; i++)
                                    //    updates.emplace_back(j.joint, LssPosition | LssAction | LssDegrees, j.position.target);
                                }

                                if (j->mmd.changed(false)) {
                                    updates.emplace_back(j->joint, LssAction | LssMaxDuty, j->mmd.target());
                                    j->mmd.current(j->mmd.target());
                                }

                                switch (j->state) {
                                    case CompliantJoint::PositiveCompliance:
                                    case CompliantJoint::NegativeCompliance:
                                        updates.emplace_back(j->joint, LssPosition | LssAction | LssDegrees,
                                                             j->position.target());
                                        break;

                                    case CompliantJoint::Holding:
                                        updates.insert(updates.end(),
                                                       LynxPacket(j->joint, LssPosition | LssAction | LssDegrees,
                                                                  j->position.target())
                                                               .currentHaltAndLimp(j->currentLimit)
                                        );
                                        break;

                                    case CompliantJoint::Moving:
                                        updates.insert(updates.end(),
                                                       LynxPacket(j->joint, LssPosition | LssAction | LssDegrees,
                                                                  j->position.target())
                                        );
                                        break;

                                    case CompliantJoint::ComplianceLimp:
                                        updates.emplace_back(j->joint, LssLimp | LssAction);
                                        break;

                                    default:
                                        break;
                                }
                                j->position.changed(false);

                            }
                            channel->send(updates.begin(), updates.end())
                                    .regardless(
                                            [this, qstart, ustart](const LssTransaction &) {
                                                //auto now = micros();
                                                //utime.add(now - ustart);
                                                success++;
                                                consecutive_failures = 0;

                                                remaining_mutex.lock();
                                                remaining--;
                                                if (remaining == 0 && _joint_state_pub->is_activated())
                                                    _joint_state_pub->publish(*_joint_state_msg);
                                                remaining_mutex.unlock();
                                            });
                        })
                        .otherwise([this](const LssTransaction &tx) {
                            if (consecutive_failures > 5 || log_bus_warnings.as_bool()) {
                                // print the state of packets in the transaction
                                printf("expired: \n");
                                char s[32];
                                unsigned long long st = 0;
                                for (auto &p: tx.packets()) {
                                    if (st == 0) st = p.microstamp;
                                    p.serialize(s);
                                    if (p.command & LssAction)
                                        printf("   %5lld: > %s\n", p.microstamp - st, s);
                                    else if (p.command & LssQuery)
                                        printf("   %5lld: %c %s\n", (p.microstamp > 0) ? p.microstamp - st : 0,
                                               p.hasValue ? '<' : '!', s);
                                }
                            }

                            failed++;
                            consecutive_failures++;

                            remaining_mutex.lock();
                            remaining--;
                            remaining_mutex.unlock();
                        });
            }

        } catch (std::exception &e) {
            RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "Failed to poll and publish data: %s", e.what());
            deactivate();
        }
    }
#endif

    /*rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LynxmotionServoHardware::disable_joints(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "LSS joint controller commanding all servos to go limp");
        CallbackReturn rv = CallbackReturn::SUCCESS;
        for (auto &channel: channels) {
            CallbackReturn l_rv;
            if ((l_rv = channel->on_deactivate(state)) != CallbackReturn::SUCCESS)
                rv = l_rv;
        }
        return rv;
    }*/


    /*void LynxmotionServoHardware::publish_diagnostics() try {

    } catch (std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Failed to poll and publish data: %s", e.what());
        deactivate();
    }*/



#if 0
    bool start_hardware_iop(LynxmotionServoHardware* iop) {
        iops.emplace_back(iop);

        if (hw_process_state == Stopped) {
            hw_process_state = Starting;
            if(pthread_create( &lss_hw_process, NULL, lss_hw_process_thread, (void*) NULL) !=0) {
                RCUTILS_LOG_ERROR_NAMED(lss_hardware_logger, "failed to start LSS hardware interface thread, no I/O will occur.");
            }
        }
        return iop;
    }

    void stop_hardware_iop(LynxmotionServoHardware* p) {
        LynxmotionRealtime& rt = LynxmotionRealtime::get_default();
        rt.remove_bus(*this);

        // find and remove iop from list
        auto p_itr = std::find(iops.begin(), iops.end(), p);
        if(p_itr != iops.end()) {
            // todo: we cant release this iop until we are sure the processing thread is done with it
            iops.erase(p_itr);

            // stop processing thread if no more iops to do
            if (iops.empty() && hw_process_state > Stopped) {
                void *rv = NULL;
                hw_process_state = Stopping;
                //pthread_cancel(priv->loop);
                pthread_join(lss_hw_process, &rv);
            }
        }
    }

    void* lss_hw_process_thread(void*) {
        size_t iop_n = 0;
        hw_process_state = Idle;
        bool sleep = false;
        while(hw_process_state >= Idle) {
            std::unique_lock<std::mutex> guard(iops_mutex, std::try_to_lock);
            if(guard.owns_lock()) {
                // get the next IOP
                if (iop_n >= iops.size()) {
                    iop_n = 0;
                    sleep = true;
                }
                LynxmotionServoHardware *hw = iops[iop_n];
                guard.unlock();

                lss_hw_spin_iop(hw);

                iop_n++;

                if(sleep) {
                    // todo: we need to setup events and wait on them instead of this polling
                    usleep(100);
                    sleep = false;
                }
            }
        }
        hw_process_state = Stopped;
        return nullptr;
    }
#endif


} //ns:lynxmotion



