#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

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

int main(int argc, char const *argv[]){
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
      }
    
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    
    unitree::robot::g1::LocoClient locoClient;
    locoClient.Init();
    locoClient.SetTimeout(10.f);

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
  
    std::cout << "Press ENTER to balanced stand. " << std::endl;
    std::cin.get();
    locoClient.BalanceStand();
    
}

