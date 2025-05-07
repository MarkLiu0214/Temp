#include <chrono>
#include <iostream>
#include <thread>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/String_.hpp>

#define AUDIO_SUBSCRIBE_TOPIC "rt/audio_msg"

// 定義ASR訊息處理函數
void asr_handler(const void *msg, unitree::robot::g1::AudioClient &audioClient, unitree::robot::g1::LocoClient &locoClient) {
    
    
    std_msgs::msg::dds_::String_ *resMsg = (std_msgs::msg::dds_::String_ *)msg;
    std::string asrData = resMsg->data();
    std::cout << "ASR回調: " << asrData << std::endl;

    std::string keyword = "text";
    size_t pos = asrData.find(keyword);
    if (pos != std::string::npos) {
        std::string messageAfterText = asrData.substr(pos + keyword.length());
        std::cout << "檢測到 'text'，後面接的訊息為: " << messageAfterText << std::endl;

        // 檢查由語音指令觸發的TTS回應
        if (messageAfterText.find("你是谁") != std::string::npos) {
            audioClient.TtsMaker("你好，我是居萬機器人", 0);
        } else if (messageAfterText.find("你在哪") != std::string::npos) {
            audioClient.TtsMaker("我目前在國立陽明交通大學。", 0);
        } else if (messageAfterText.find("你来自") != std::string::npos) {
            audioClient.TtsMaker("我來自國立陽明交通大學", 0);
        }
        else if(messageAfterText.find("前进") != std::string::npos){
            audioClient.TtsMaker("前進", 0);
            locoClient.SetVelocity(0.3, 0, 0, 3);
            std::this_thread::sleep_for(std::chrono::seconds(5));
        } 
        else if(messageAfterText.find("后退") != std::string::npos){
            audioClient.TtsMaker("後退", 0);
            locoClient.SetVelocity(-0.3, 0, 0, 3);
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        else if(messageAfterText.find("右转") != std::string::npos){
            audioClient.TtsMaker("右轉", 0);
            locoClient.SetVelocity(0.0f, 0.0f, -0.5f, 3);
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        else if(messageAfterText.find("左转") != std::string::npos){
            audioClient.TtsMaker("左轉", 0);
            locoClient.SetVelocity(0.0f, 0.0f, 0.5f, 3);
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        // 可根據需求增加更多的語音指令處理
    }
}

int main(int argc, char const *argv[]) {
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

    // 初始化語音客戶端
    unitree::robot::g1::AudioClient audioClient;
    audioClient.Init();
    audioClient.SetTimeout(10.0f);

    // 註冊 ASR 回調函數
    unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(AUDIO_SUBSCRIBE_TOPIC);
    subscriber.InitChannel(std::bind(asr_handler, std::placeholders::_1, std::ref(audioClient), std::ref(locoClient)));

    // 移動控制提示
    std::cout << "啟動遙控命令，請輸入 'w、a、s、d、z、c' 控制移動方位，或輸入'q' 退出。" << std::endl;

    // 讀取鍵盤輸入實現機器人運動控制循環
    while (true) {
        char input;
        std::cin >> input;
        if (input == 'w') {
            locoClient.Move(0.3f, 0.0f, 0.0f); // 前進
        } else if (input == 'a') {
            locoClient.Move(0.0f, 0.0f, 0.5f); // 左轉
        } else if (input == 's') {
            locoClient.Move(-0.3f, 0.0f, 0.0f); // 後退
        } else if (input == 'd') {
            locoClient.Move(0.0f, 0.0f, -0.5f); // 右轉
        } else if (input =='z') {
            locoClient.Move(0.0f, 0.3f, 0.0f); // 左移
        } else if (input == 'c') {
            locoClient.Move(0.0f, -0.3f, 0.0f); // 右移
        } else if (input == 'q') {
            std::cout << "退出程序。" << std::endl;
            break; // 退出循環
        } else {
            std::cout << "無效指令。請按 'w、a、s、d、z、c' 控制移動方位，'q' 退出。" << std::endl;
        }
    }

    return 0;
}
