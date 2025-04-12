#include <array>
#include <chrono>
#include <iostream>
#include <thread>

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

  // wait for init
  std::cout << "Press ENTER to init arms ...";
  std::cin.get();

  // get current joint position
  std::array<float, 17> current_jpos{};
  std::cout<<"Current joint position: ";
  for (int i = 0; i < arm_joints.size(); ++i) {
	current_jpos.at(i) = state_msg.motor_state().at(arm_joints.at(i)).q();
	std::cout << current_jpos.at(i) << " ";
  }
  std::cout << std::endl;

  // set init pos
  std::cout << "Initailizing arms ...";
  float init_time = 2.0f;
  int init_time_steps = static_cast<int>(init_time / control_dt);

  for (int i = 0; i < init_time_steps; ++i) {
    // increase weight
    weight = 1.0;
    msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
    float phase = 1.0 * i / init_time_steps;
    std::cout << "Phase: " << phase << std::endl;

    // set control joints
    for (int j = 0; j < init_pos.size(); ++j) {
      msg.motor_cmd().at(arm_joints.at(j)).q(init_pos.at(j) * phase + current_jpos.at(j) * (1 - phase));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    // send dds msg
    arm_sdk_publisher->Write(msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Done!" << std::endl;

    // 在 main 裡面的 arm control 區段換成這段即可
    std::cout << "Press ENTER to start waving..." << std::endl;
    std::cin.get();

    std::cout << "Start waving!" << std::endl;

    float wave_duration = 5.0f;  // 揮手時間（秒）
    int wave_steps = static_cast<int>(wave_duration / control_dt);
    float wave_freq = 1.0f;      // 揮手頻率（Hz）
    float wave_amp = 0.5f;       // 揮手振幅（弧度）

    for (int i = 0; i < wave_steps; ++i) {
        float t = i * control_dt;
        float wave_angle = wave_amp * std::sin(2 * kPi * wave_freq * t);

        // 把其他關節保持不動，只動 RightWistYaw
        for (int j = 0; j < arm_joints.size(); ++j) {
            JointIndex joint = arm_joints.at(j);
            float q_des = init_pos.at(j); // default to init position

            if (joint == JointIndex::kRightWistYaw) {
                q_des = wave_angle;
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

std::cout << "Waving done!" << std::endl;

}
