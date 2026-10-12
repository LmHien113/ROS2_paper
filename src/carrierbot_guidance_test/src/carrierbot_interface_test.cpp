#include "carrierbot_guidance_test/carrierbot_interface_test.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <cstring>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <sstream>

namespace carrierbot_guidance_test
{
    CarrierbotInterfaceTest::CarrierbotInterfaceTest() = default;

    CarrierbotInterfaceTest::~CarrierbotInterfaceTest()
    {
        stopPollLoop();
        if (can_interface_ != nullptr)
        {
            try
            {
                sendZeroCommand();
                can_interface_->close();
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterfaceTest"),
                                    "Failed to close CAN port: " << e.what());
            }
            can_interface_.reset();
        }
    }

    int CarrierbotInterfaceTest::convertPulse(double velocity_mps)
    {
        // v1 ConvertPulse: PPR * v(m/s) * 1000 / (2*pi*r_mm)
        return static_cast<int>(
            kPulsePerRevolution * velocity_mps * 1000.0 / (2.0 * kPi * kWheelRadiusMm));
    }

    double CarrierbotInterfaceTest::convertVelocityFromPulse(int pulse)
    {
        const double circumference_mm = 2.0 * kPi * kWheelRadiusMm;
        return (static_cast<double>(pulse) * circumference_mm) /
               (static_cast<double>(kPulsePerRevolution) * 1000.0);
    }

    hardware_interface::return_type CarrierbotInterfaceTest::configure(
        const hardware_interface::HardwareInfo &info)
    {
        info_ = info;

        try
        {
            port_ = info_.hardware_parameters.at("port");
            baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
        }
        catch (const std::out_of_range &)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CarrierbotInterfaceTest"), "No CAN port specified!");
            return hardware_interface::return_type::ERROR;
        }

        if (info_.hardware_parameters.count("drive_mode"))
        {
            drive_mode_ = std::stoi(info_.hardware_parameters.at("drive_mode"));
        }
        if (info_.hardware_parameters.count("poll_period"))
        {
            poll_period_sec_ = std::stod(info_.hardware_parameters.at("poll_period"));
        }
        // Dau cua tung banh (+1 hoac -1). Mac dinh giong ban goc: banh trai dao dau.
        const auto read_sign = [this](const std::string &name, double fallback) {
            if (!info_.hardware_parameters.count(name))
            {
                return fallback;
            }
            return std::stod(info_.hardware_parameters.at(name)) < 0.0 ? -1.0 : 1.0;
        };
        left_cmd_sign_ = read_sign("left_command_sign", -1.0);
        right_cmd_sign_ = read_sign("right_command_sign", 1.0);
        left_fb_sign_ = read_sign("left_feedback_sign", -1.0);
        right_fb_sign_ = read_sign("right_feedback_sign", 1.0);

        // Same RFID table as v1 can_node.cpp
        rfid_database_ = {
            {{0xd2, 0x0f, 0x49, 0x2e, 0xba, 0x55, 0xaa, 0xc8}, {"HOAI PHU", "Phu"}},
            {{0xd2, 0xb1, 0x3d, 0x05, 0x5b, 0x55, 0xaa, 0xc8}, {"MINH KY", "Ky"}},
            {{0xfa, 0xdc, 0x02, 0xcd, 0xe9, 0x55, 0xaa, 0xc8}, {"QUANG DUY", "Duy"}},
            {{0xef, 0xa8, 0x98, 0x1e, 0xc1, 0x55, 0xaa, 0xc8}, {"CHI THIEN", "Thien"}},
            {{0xb6, 0x87, 0x13, 0x2b, 0x09, 0x55, 0xaa, 0xc8}, {"VAN LOI", "Loi"}},
            {{0xc2, 0xbf, 0xb0, 0x2e, 0xe3, 0x55, 0xaa, 0xc8}, {"BACH THU", "Thu"}},
            {{0xd2, 0xb8, 0x3d, 0x04, 0x5b, 0x55, 0xaa, 0xc8}, {"DINH HUY", "Huy"}},
        };

        can_interface_ = std::make_unique<WaveshareCAN>(port_, baudrate_, 2.0);
        ros_node_ = std::make_shared<rclcpp::Node>("carrierbot_interface_test");
        telemetry_pub_ = ros_node_->create_publisher<carrierbot_msgs::msg::CarrierbotTelemetry>(
            "/carrierbot/telemetry", 10);
        rfid_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
            "/carrierbot/rfid", 10);
        rotate_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32>(
            "/carrierbot/rotate_angle", 10,
            std::bind(&CarrierbotInterfaceTest::onRotateAngle, this, std::placeholders::_1));

        velocity_command_.assign(info_.joints.size(), 0.0);
        position_state_.assign(info_.joints.size(), 0.0);
        velocity_state_.assign(info_.joints.size(), 0.0);

        RCLCPP_INFO(
            rclcpp::get_logger("CarrierbotInterfaceTest"),
            "[TEST] Configured hardware: port=%s baud=%d mode=%d poll=%.3fs R=%.3fm "
            "sign cmd L=%+.0f R=%+.0f, feedback L=%+.0f R=%+.0f",
            port_.c_str(), baudrate_, drive_mode_, poll_period_sec_, kWheelRadiusM,
            left_cmd_sign_, right_cmd_sign_, left_fb_sign_, right_fb_sign_);

        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface>
    CarrierbotInterfaceTest::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_state_[i]);
            state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_state_[i]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    CarrierbotInterfaceTest::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_command_[i]);
        }
        return command_interfaces;
    }

    void CarrierbotInterfaceTest::sendCan(uint16_t can_id, const std::vector<uint8_t> &data)
    {
        if (!can_interface_)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(can_mutex_);
        can_interface_->send(can_id, data);
    }

    void CarrierbotInterfaceTest::sendDriveMode(int mode)
    {
        std::vector<uint8_t> data(8, 0);
        data[0] = static_cast<uint8_t>(mode);
        sendCan(0x020, data);
        RCLCPP_INFO(rclcpp::get_logger("CarrierbotInterfaceTest"), "Sent drive mode %d (CAN 0x020)", mode);
    }

    void CarrierbotInterfaceTest::sendZeroCommand()
    {
        std::vector<uint8_t> data(8, 0);
        sendCan(0x030, data);
    }

    void CarrierbotInterfaceTest::sendRotateAngle(double angle_deg)
    {
        const double clamped = std::max(
            0.0,
            std::min(angle_deg, static_cast<double>(std::numeric_limits<uint16_t>::max())));
        const uint16_t encoded = static_cast<uint16_t>(clamped);

        std::vector<uint8_t> data(8, 0);
        data[0] = static_cast<uint8_t>(encoded & 0xFF);
        data[1] = static_cast<uint8_t>((encoded >> 8) & 0xFF);
        sendCan(0x040, data);

        RCLCPP_INFO(
            rclcpp::get_logger("CarrierbotInterfaceTest"),
            "Sent rotate angle %.2f (CAN value=%u degrees) via CAN 0x040",
            angle_deg, static_cast<unsigned int>(encoded));
    }

    void CarrierbotInterfaceTest::onRotateAngle(const std_msgs::msg::Float32::SharedPtr msg)
    {
        try
        {
            sendRotateAngle(msg->data);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(
                rclcpp::get_logger("CarrierbotInterfaceTest"),
                "Failed to send rotate CAN 0x040: " << e.what());
        }
    }

    void CarrierbotInterfaceTest::startPollLoop()
    {
        if (poll_running_)
        {
            return;
        }
        poll_running_ = true;
        poll_thread_ = std::thread([this]() {
            using namespace std::chrono_literals;
            while (poll_running_)
            {
                try
                {
                    sendCan(0x050, {1, 0, 0, 0, 0, 0, 0, 0});
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN_STREAM(
                        rclcpp::get_logger("CarrierbotInterfaceTest"),
                        "CAN poll 0x050 failed: " << e.what());
                }
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(poll_period_sec_));
            }
        });
    }

    void CarrierbotInterfaceTest::stopPollLoop()
    {
        poll_running_ = false;
        if (poll_thread_.joinable())
        {
            poll_thread_.join();
        }
    }

    hardware_interface::return_type CarrierbotInterfaceTest::start()
    {
        RCLCPP_INFO(rclcpp::get_logger("CarrierbotInterfaceTest"), "Starting v1 robot hardware");
        if (!can_interface_)
        {
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            can_interface_->open();
            auto callback = [this](uint16_t can_id, const std::vector<uint8_t> &data)
            {
                if (can_id == 0x011)
                {
                    handleEncoderData(data);
                }
                else if (can_id == 0x019)
                {
                    handleRfidData(data);
                }
            };
            can_interface_->start_receive_loop(callback);

            // Match robot_fablab_ws launch: send the configured hardware drive mode.
            sendDriveMode(drive_mode_);
            startPollLoop();
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterfaceTest"),
                                "Cannot open CAN: " << e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CarrierbotInterfaceTest::stop()
    {
        RCLCPP_INFO(rclcpp::get_logger("CarrierbotInterfaceTest"), "Stopping robot hardware");
        stopPollLoop();
        try
        {
            if (can_interface_)
            {
                sendZeroCommand();
                can_interface_->close();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterfaceTest"),
                                "Failed to close CAN port: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    void CarrierbotInterfaceTest::handleEncoderData(const std::vector<uint8_t> &data)
    {
        if (data.size() < 8)
        {
            return;
        }

        int left_pulse = 0;
        int right_pulse = 0;
        std::memcpy(&left_pulse, &data[0], sizeof(int));
        std::memcpy(&right_pulse, &data[4], sizeof(int));

        std::lock_guard<std::mutex> lock(state_mutex_);
        raw_left_pulse_s_ = left_pulse;
        raw_right_pulse_s_ = right_pulse;
    }

    void CarrierbotInterfaceTest::handleRfidData(const std::vector<uint8_t> &data)
    {
        auto it = rfid_database_.find(data);
        if (it == rfid_database_.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("CarrierbotInterfaceTest"), "Unknown RFID tag");
            return;
        }

        const std::string &full_name = it->second.first;
        const std::string &short_name = it->second.second;

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_time), "%H:%M:%S");
        const std::string timestamp = ss.str();

        // Payload matches v1 name_publisher.py fields
        std_msgs::msg::String msg;
        msg.data = "{\"message\":\"" + short_name +
                   "\",\"user\":\"" + full_name +
                   "\",\"time\":\"" + timestamp + "\"}";

        if (rfid_pub_)
        {
            rfid_pub_->publish(msg);
        }

        RCLCPP_INFO(
            rclcpp::get_logger("CarrierbotInterfaceTest"),
            "RFID detected: %s (%s) at %s",
            full_name.c_str(), short_name.c_str(), timestamp.c_str());
    }

    hardware_interface::return_type CarrierbotInterfaceTest::read()
    {
        // Process rotate MQTT→ROS subscriptions without a dedicated spin thread
        if (ros_node_)
        {
            rclcpp::spin_some(ros_node_);
        }

        rclcpp::Time now = rclcpp::Clock().now();
        if (last_run_.nanoseconds() == 0)
        {
            last_run_ = now;
            return hardware_interface::return_type::OK;
        }

        double dt = (now - last_run_).seconds();
        if (dt <= 0.0 || dt > 1.0)
        {
            last_run_ = now;
            return hardware_interface::return_type::OK;
        }

        int left_pulse = 0;
        int right_pulse = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            left_pulse = raw_left_pulse_s_;
            right_pulse = raw_right_pulse_s_;
        }

        // v1 updateWheelOdometry scale: v_l = -left/20, v_r = right/20 (m/s)
        // Dau encoder doc ve (0x011) cau hinh duoc: left_feedback_sign / right_feedback_sign
        const double left_mps = left_fb_sign_ * convertVelocityFromPulse(left_pulse) / kVelocityScale;
        const double right_mps = right_fb_sign_ * convertVelocityFromPulse(right_pulse) / kVelocityScale;

        const double left_rad_s = left_mps / kWheelRadiusM;
        const double right_rad_s = right_mps / kWheelRadiusM;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Joint order in ros2_control.xacro: [0]=right, [1]=left
            velocity_state_[0] = right_rad_s;
            velocity_state_[1] = left_rad_s;
            telemetry_msg_.left_rps = static_cast<float>(left_rad_s / (2.0 * kPi));
            telemetry_msg_.right_rps = static_cast<float>(right_rad_s / (2.0 * kPi));
            telemetry_msg_.left_velocity = static_cast<float>(left_mps);
            telemetry_msg_.right_velocity = static_cast<float>(right_mps);
            telemetry_msg_.left_measured_mps = static_cast<float>(left_mps);
            telemetry_msg_.right_measured_mps = static_cast<float>(right_mps);
        }

        position_state_[0] += velocity_state_[0] * dt;
        position_state_[1] += velocity_state_[1] * dt;
        publishTelemetry();

        last_run_ = now;
        return hardware_interface::return_type::OK;
    }

    void CarrierbotInterfaceTest::sendWheelVelocities()
    {
        if (!can_interface_ || velocity_command_.size() < 2)
        {
            return;
        }

        double right_cmd_rad_s = 0.0;
        double left_cmd_rad_s = 0.0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            right_cmd_rad_s = velocity_command_[0];
            left_cmd_rad_s = velocity_command_[1];
        }

        // rad/s -> m/s, then apply v1 ×20 before pulse conversion
        // Dau lenh gui xuong STM32 (0x030) cau hinh duoc: left_command_sign / right_command_sign
        const double left_mps = left_cmd_sign_ * left_cmd_rad_s * kWheelRadiusM;
        const double right_mps = right_cmd_sign_ * right_cmd_rad_s * kWheelRadiusM;
        const int left_pulse = convertPulse(left_mps * kVelocityScale);
        const int right_pulse = convertPulse(right_mps * kVelocityScale);

        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("CarrierbotInterfaceTest"),
            *ros_node_->get_clock(),
            1000,
            "CAN 0x030 wheel command: left=%.3f rad/s (%d pulse/s), "
            "right=%.3f rad/s (%d pulse/s)",
            left_cmd_rad_s,
            left_pulse,
            right_cmd_rad_s,
            right_pulse);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            telemetry_msg_.left_velocity = static_cast<float>(left_mps);
            telemetry_msg_.right_velocity = static_cast<float>(right_mps);
            telemetry_msg_.left_command_mps = static_cast<float>(left_mps);
            telemetry_msg_.right_command_mps = static_cast<float>(right_mps);
        }
        publishTelemetry();

        uint8_t data[8];
        std::memcpy(data, &left_pulse, sizeof(int));
        std::memcpy(data + 4, &right_pulse, sizeof(int));
        sendCan(0x030, std::vector<uint8_t>(data, data + 8));
    }

    hardware_interface::return_type CarrierbotInterfaceTest::write()
    {
        try
        {
            sendWheelVelocities();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("CarrierbotInterfaceTest"),
                                "Failed to send CAN command: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    std::string CarrierbotInterfaceTest::get_name() const
    {
        return info_.name;
    }

    hardware_interface::status CarrierbotInterfaceTest::get_status() const
    {
        return hardware_interface::status::STARTED;
    }

    void CarrierbotInterfaceTest::publishTelemetry()
    {
        if (!telemetry_pub_)
        {
            return;
        }
        telemetry_pub_->publish(telemetry_msg_);
    }

}  // namespace carrierbot_guidance_test

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(carrierbot_guidance_test::CarrierbotInterfaceTest,
                       hardware_interface::SystemInterface)
