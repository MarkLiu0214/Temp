#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";
constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;

enum JointIndex {
    // Left leg
    kLeftHipPitch,
    kLeftHipRoll,
    kLeftHipYaw,
    kLeftKnee,
    kLeftAnkle,
    kLeftAnkleRoll,

    // Right leg
    kRightHipPitch,
    kRightHipRoll,
    kRightHipYaw,
    kRightKnee,
    kRightAnkle,
    kRightAnkleRoll,

    kWaistYaw,
    kWaistRoll,
    kWaistPitch,

    // Left arm
    kLeftShoulderPitch,
    kLeftShoulderRoll,
    kLeftShoulderYaw,
    kLeftElbow,
    kLeftWistRoll,
    kLeftWistPitch,
    kLeftWistYaw,
    // Right arm
    kRightShoulderPitch,
    kRightShoulderRoll,
    kRightShoulderYaw,
    kRightElbow,
    kRightWistRoll,
    kRightWistPitch,
    kRightWistYaw,

    kNotUsedJoint,
    kNotUsedJoint1,
    kNotUsedJoint2,
    kNotUsedJoint3,
    kNotUsedJoint4,
    kNotUsedJoint5
};

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
      arm_sdk_publisher;
  unitree_hg::msg::dds_::LowCmd_ msg;

  arm_sdk_publisher.reset(
      new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
          kTopicArmSDK));
  arm_sdk_publisher->InitChannel();

  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
      low_state_subscriber;

  // create subscriber
  unitree_hg::msg::dds_::LowState_ state_msg;
  low_state_subscriber.reset(
      new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
          kTopicState));
  low_state_subscriber->InitChannel([&](const void *msg) {
        auto s = ( const unitree_hg::msg::dds_::LowState_* )msg;
        memcpy( &state_msg, s, sizeof( unitree_hg::msg::dds_::LowState_ ) );
  }, 1);

  std::array<JointIndex, 17> arm_joints = {
      JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
      JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
      JointIndex::kLeftWistRoll,       JointIndex::kLeftWistPitch,
      JointIndex::kLeftWistYaw,
      JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
      JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow,
      JointIndex::kRightWistRoll,      JointIndex::kRightWistPitch,
      JointIndex::kRightWistYaw,
      JointIndex::kWaistYaw,
      JointIndex::kWaistRoll,
      JointIndex::kWaistPitch};

  float weight = 0.f;
  float weight_rate = 0.2f;

  float kp = 60.f;
  float kd = 1.5f;
  float dq = 0.f;
  float tau_ff = 0.f;

  float control_dt = 0.02f;
  float max_joint_velocity = 0.5f;

  float delta_weight = weight_rate * control_dt;
  float max_joint_delta = max_joint_velocity * control_dt;
  auto sleep_time =
      std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

  std::array<float, 17> init_pos{0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0};

  std::array<float, 17> target_pos = {0.f, kPi_2,  0.f, kPi_2, 0.f, 0.f, 0.f,
                                     0.f, -kPi_2, 0.f, kPi_2, 0.f, 0.f, 0.f, 
                                     0, 0, 0};

// 在 main 裡面的 arm control 區段換成這段即可
// === Step 1: 把右手慢慢舉起來 ===
std::array<float, 17> current_jpos_des{};
for (int i = 0; i < num_time_steps; ++i) {
    for (int j = 0; j < init_pos.size(); ++j) {
        current_jpos_des.at(j) += std::clamp(target_pos.at(j) - current_jpos_des.at(j),
                                             -max_joint_delta, max_joint_delta);
    }

    for (int j = 0; j < init_pos.size(); ++j) {
        msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
        msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
        msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
        msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
        msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    arm_sdk_publisher->Write(msg);
    std::this_thread::sleep_for(sleep_time);
}

// === Step 2: 揮手 (整隻手臂左右搖) ===
std::cout << "Start waving with shoulder!" << std::endl;

float wave_duration = 5.0f;
int wave_steps = static_cast<int>(wave_duration / control_dt);
float wave_freq = 1.0f;
float wave_amp = 0.5f;

for (int i = 0; i < wave_steps; ++i) {
    float t = i * control_dt;
    float wave_angle = wave_amp * std::sin(2 * kPi * wave_freq * t);

    for (int j = 0; j < arm_joints.size(); ++j) {
        JointIndex joint = arm_joints.at(j);
        float q_des = current_jpos_des.at(j); // 保持原來姿勢

        if (joint == JointIndex::kRightShoulderYaw) {
            q_des = wave_angle; // 改為控制肩膀的橫向搖擺
        }

        msg.motor_cmd().at(joint).q(q_des);
        msg.motor_cmd().at(joint).dq(dq);
        msg.motor_cmd().at(joint).kp(kp);
        msg.motor_cmd().at(joint).kd(kd);
        msg.motor_cmd().at(joint).tau(tau_ff);
    }

    arm_sdk_publisher->Write(msg);
    std::this_thread::sleep_for(sleep_time);
}


// === Step 3: 把右手慢慢放下回到初始姿勢 ===
for (int i = 0; i < num_time_steps; ++i) {
    for (int j = 0; j < init_pos.size(); ++j) {
        current_jpos_des.at(j) += std::clamp(init_pos.at(j) - current_jpos_des.at(j),
                                             -max_joint_delta, max_joint_delta);
    }

    for (int j = 0; j < init_pos.size(); ++j) {
        msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
        msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
        msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
        msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
        msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    arm_sdk_publisher->Write(msg);
    std::this_thread::sleep_for(sleep_time);
}

std::cout << "Arm returned to init position." << std::endl;

d

}
