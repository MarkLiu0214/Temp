#include <iostream>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <chrono>
#include <thread>


int  main(int argc, char const *argv[]){
    if (argc < 2) {
        std::cout << "使用方法: program_name [network_interface，如 eth0]" << std::endl;
        return 1;
    }
    
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

    char input;
    std::cin >> input;
    if(input == 's'){
        locoClient.SetVelocity(0.1, 0, 0, 3);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        locoClient.SetVelocity(0, 0, -0.3, 3);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        locoClient.SetVelocity(0.1, 0, 0, 3);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        locoClient.SetVelocity(0, 0, 0.3, 3);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    


    


}
