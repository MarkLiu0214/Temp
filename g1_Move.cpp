#include <iostream>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <chrono>
#include <thread>


int  main(int argc, char const *argv[]){
    // 初始化通道工廠
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    // 初始化機器人移動客戶端
    unitree::robot::g1::LocoClient locoClient;
    locoClient.Init();
    locoClient.SetTimeout(10.f);

    /*
    locoClient.Move(0.3f, 0, 0, true);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    */

    locoClient.SetVelocity(0.3, 0, 0, 3);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    locoClient.SetVelocity(0, 0, -0.3, 3);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    locoClient.SetVelocity(0.3, 0, 0, 3);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    locoClient.SetVelocity(0, 0, 0.3, 3);
    std::this_thread::sleep_for(std::chrono::seconds(1));



}
