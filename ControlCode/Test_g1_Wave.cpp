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

void RaiseRightHand(unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>& pub,
  unitree_hg::msg::dds_::LowCmd_& msg,
  const unitree_hg::msg::dds_::LowState_& state,
  const std::array<JointIndex, 17>& arm_joints,
  float kp, float kd, float tau_ff,
  float control_dt, float max_joint_velocity) {

  std::array<float, 17> current_jpos{};
  for (int i = 0; i < arm_joints.size(); ++i) {
    current_jpos.at(i) = state.motor_state().at(arm_joints.at(i)).q();
  }

  // 目標姿勢：右手抬起來（你可以微調這些角度）
  std::array<float, 17> target_pos = current_jpos;
  target_pos[7] = 0.0f;     // RightShoulderPitch
  target_pos[8] = -1.0f;    // RightShoulderRoll
  target_pos[9] = 0.5f;     // RightShoulderYaw
  target_pos[10] = 1.2f;    // RightElbow

  int steps = static_cast<int>(2.0f / control_dt);
  float max_joint_delta = max_joint_velocity * control_dt;

  for (int i = 0; i < steps; ++i) {
    for (int j = 0; j < arm_joints.size(); ++j) {
      float delta = std::clamp(target_pos.at(j) - current_jpos.at(j),
                        -max_joint_delta, max_joint_delta);
      current_jpos.at(j) += delta;

      msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(0.f);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    pub->Write(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(
    static_cast<int>(control_dt * 1000)));
  }
}

void WaveRightHand(unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>& pub,
 unitree_hg::msg::dds_::LowCmd_& msg,
 const std::array<JointIndex, 17>& arm_joints,
 std::array<float, 17>& base_pos,
 float kp, float kd, float tau_ff,
 float control_dt, float duration = 3.0f) {

  float freq = 1.5f;
  float amp = 0.5f;
  int steps = static_cast<int>(duration / control_dt);

  for (int i = 0; i < steps; ++i) {
    float t = i * control_dt;
    float wave = amp * std::sin(2 * kPi * freq * t);

    for (int j = 0; j < arm_joints.size(); ++j) {
      float q_des = base_pos.at(j);

      if (arm_joints.at(j) == JointIndex::kRightShoulderYaw ||
      arm_joints.at(j) == JointIndex::kRightElbow) {
      q_des += wave;
      }

      msg.motor_cmd().at(arm_joints.at(j)).q(q_des);
      msg.motor_cmd().at(arm_joints.at(j)).dq(0.f);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    pub->Write(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(
    static_cast<int>(control_dt * 1000)));
  }
}

void LowerRightHand(unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>& pub,
  unitree_hg::msg::dds_::LowCmd_& msg,
  const unitree_hg::msg::dds_::LowState_& state,
  const std::array<JointIndex, 17>& arm_joints,
  float kp, float kd, float tau_ff,
  float control_dt, float max_joint_velocity) {

  std::array<float, 17> current_jpos{};
  for (int i = 0; i < arm_joints.size(); ++i) {
    current_jpos.at(i) = state.motor_state().at(arm_joints.at(i)).q();
  }

  std::array<float, 17> target_pos = current_jpos;
  target_pos[7] = 0;  // RightShoulderPitch
  target_pos[8] = 0;
  target_pos[9] = 0;
  target_pos[10] = 0;

  int steps = static_cast<int>(2.0f / control_dt);
  float max_joint_delta = max_joint_velocity * control_dt;

  for (int i = 0; i < steps; ++i) {
    for (int j = 0; j < arm_joints.size(); ++j) {
      float delta = std::clamp(target_pos.at(j) - current_jpos.at(j),
                        -max_joint_delta, max_joint_delta);
      current_jpos.at(j) += delta;

      msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(0.f);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    pub->Write(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(
    static_cast<int>(control_dt * 1000)));
  }
}



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
  float period = 5.;
  int num_time_steps = static_cast<int>(period / control_dt);
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

  std::cout << "Press ENTER to start waving..." << std::endl;
  std::cin.get();

  RaiseRightHand(arm_sdk_publisher, msg, state_msg, arm_joints,
                kp, kd, tau_ff, control_dt, max_joint_velocity);

  std::cout << "Waving!" << std::endl;
  WaveRightHand(arm_sdk_publisher, msg, arm_joints,
                current_jpos_des, kp, kd, tau_ff, control_dt);

  LowerRightHand(arm_sdk_publisher, msg, state_msg, arm_joints,
                kp, kd, tau_ff, control_dt, max_joint_velocity);

  std::cout << "Done waving 👋" << std::endl;

  // stop control
  std::cout << "Press Enter to stop arm ctrl ...";
  std::cin.get();
  float stop_time = 2.0f;
  int stop_time_steps = static_cast<int>(stop_time / control_dt);
  
  for (int i = 0; i < stop_time_steps; ++i) {
      // increase weight
      weight -= delta_weight;
      weight = std::clamp(weight, 0.f, 1.f);
  
      // 设置电机转动的平滑增减权重
      // 电机将逐渐进入自由状态
      msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
  
      // 发送上肢电机的控制指令
      arm_sdk_publisher->Write(msg);
  
      // 延时一个控制步长
      std::this_thread::sleep_for(sleep_time);
  }
  
  std::cout << "Stop arm control. " << std::endl;



}
