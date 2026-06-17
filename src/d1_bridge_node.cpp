#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

using namespace unitree::robot;

static constexpr const char* DDS_SERVO_TOPIC = "current_servo_angle";
static constexpr const char* DDS_CMD_TOPIC   = "rt/arm_Command";
static constexpr const char* ROS_STATE_TOPIC = "/d1/joint_states";
static constexpr const char* ROS_CMD_TOPIC   = "/d1/joint_commands";

// Names match the IsaacLab training env joint names exactly.
// Joint1-Joint6 = arm (servo0-servo5), Joint_L/Joint_R = gripper (both driven by servo6).
static constexpr std::array<const char*, 8> JOINT_NAMES = {
    "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "Joint_L", "Joint_R"
};

class D1BridgeNode : public rclcpp::Node {
public:
    D1BridgeNode() : Node("d1_bridge_node"), seq_(0), has_prev_(false) {
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(ROS_STATE_TOPIC, 10);

        joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            ROS_CMD_TOPIC, 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_command(msg); });

        arm_pub_ = std::make_shared<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(DDS_CMD_TOPIC);
        arm_pub_->InitChannel();

        // DDS callback runs in a separate CycloneDDS thread
        servo_sub_ = std::make_shared<ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(DDS_SERVO_TOPIC);
        servo_sub_->InitChannel([this](const void* msg) { on_servo_feedback(msg); });

        RCLCPP_INFO(get_logger(), "D1 bridge ready. Listening on DDS '%s'.", DDS_SERVO_TOPIC);
    }

private:
    void on_servo_feedback(const void* raw) {
        if (!active_.load(std::memory_order_relaxed)) return;
        const auto* pm = static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw);

        // 7 hardware servo channels → 8 sim joints.
        // servo0-5: Joint1-Joint6 (arm). servo6: gripper, split symmetrically to Joint_L/Joint_R.
        std::array<double, 8> pos_deg = {
            pm->servo0_data_(), pm->servo1_data_(), pm->servo2_data_(),
            pm->servo3_data_(), pm->servo4_data_(), pm->servo5_data_(),
            pm->servo6_data_(),   // Joint_L
            pm->servo6_data_()    // Joint_R (same channel)
        };

        auto now = get_clock()->now();
        std::array<double, 8> vel_rads{};
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (has_prev_) {
                double dt = (now - prev_time_).seconds();
                if (dt > 1e-6) {
                    for (int i = 0; i < 8; ++i)
                        vel_rads[i] = (pos_deg[i] - prev_pos_deg_[i]) * M_PI / 180.0 / dt;
                }
            }
            prev_pos_deg_ = pos_deg;
            prev_time_    = now;
            has_prev_     = true;
        }

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now;
        for (int i = 0; i < 8; ++i) {
            msg.name.push_back(JOINT_NAMES[i]);
            msg.position.push_back(pos_deg[i] * M_PI / 180.0);
            msg.velocity.push_back(vel_rads[i]);
        }
        joint_state_pub_->publish(msg);
    }

    void on_joint_command(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 7) {
            RCLCPP_WARN(get_logger(), "Joint command has %zu positions, expected 7 — ignoring.",
                        msg->position.size());
            return;
        }

        // Policy outputs 8 joint positions in radians (Joint1-6, Joint_L, Joint_R).
        // D1 JSON expects 7 angles in degrees; gripper channel (angle6) = average of Joint_L/R.
        auto deg = [](double rad) { return rad * 180.0 / M_PI; };
        double gripper_deg = (deg(msg->position[6]) + deg(msg->position[7])) * 0.5;

        char json[512];
        std::snprintf(json, sizeof(json),
            "{\"seq\":%d,\"address\":1,\"funcode\":2,\"data\":"
            "{\"mode\":1"
            ",\"angle0\":%.3f,\"angle1\":%.3f,\"angle2\":%.3f"
            ",\"angle3\":%.3f,\"angle4\":%.3f,\"angle5\":%.3f,\"angle6\":%.3f}}",
            seq_++,
            deg(msg->position[0]), deg(msg->position[1]), deg(msg->position[2]),
            deg(msg->position[3]), deg(msg->position[4]), deg(msg->position[5]),
            gripper_deg);

        unitree_arm::msg::dds_::ArmString_ dds_msg{};
        dds_msg.data_() = json;
        arm_pub_->Write(dds_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;

    std::shared_ptr<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>> arm_pub_;
    std::shared_ptr<ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>> servo_sub_;

    std::atomic<bool> active_{true};
    std::mutex state_mutex_;
    std::array<double, 8> prev_pos_deg_{};
    rclcpp::Time prev_time_{0, 0, RCL_ROS_TIME};
    bool has_prev_;
    int seq_;
};

int main(int argc, char* argv[]) {
    // Must initialize Unitree DDS before rclcpp::init() — both use CycloneDDS and
    // ChannelFactory::Init() is not idempotent when called after rclcpp already owns the domain.
    ChannelFactory::Instance()->Init(0, "enx0c3796c78061");
    rclcpp::init(argc, argv);
    // rclcpp's default SIGINT/SIGTERM handler calls rclcpp::shutdown(), which triggers
    // CycloneDDS teardown that conflicts with the Unitree SDK's participant and corrupts
    // the heap. Override it AFTER rclcpp::init() to just _exit() instead.
    signal(SIGINT,  [](int) { _exit(0); });
    signal(SIGTERM, [](int) { _exit(0); });

    rclcpp::spin(std::make_shared<D1BridgeNode>());
    _exit(0);
}
