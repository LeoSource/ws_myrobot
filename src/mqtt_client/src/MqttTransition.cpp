#include "MqttTransition.h"


MqttTransition* MqttTransition::pthis = nullptr;

int MqttTransition::Connect()
{
    try
    {
        std::cout<<"Connecting to the MQTT server ..."<<std::flush;
        _client.connect(_connOpts,nullptr,_cb);
    }
    catch(const mqtt::exception& exc)
    {
        std::cerr << "\nERROR: Unable to connect to MQTT Server: '" 
        <<SERVER_ADDRESS<<"'"<<exc<<std::endl;
        return 1;
    }
}

int MqttTransition::DisConnect()
{
    try
    {
        std::cout<<"\nDisconnecting from the MQTT server..."<<std::flush;
        _client.disconnect()->wait();
        std::cout<<"OK"<<std::endl;
    }
    catch(const mqtt::exception& exc)
    {
        std::cerr << exc << std::endl;
        return 1;
    }
}

bool MqttTransition::IsConnected()
{
    return _client.is_connected();
}

void MqttTransition::GetJointPos(std::vector<double>& jpos)
{
    jpos = _cb._jpos;
}

void MqttTransition::StartRecordData()
{
    json j_text;
    j_text["record"] = "start";
    std::string pub_str = j_text.dump();
    PubTopic("jw/robot_arm/lzx",pub_str);
}

void MqttTransition::StopRecordData()
{
    json j_text;
    j_text["record"] = "stop";
    std::string pub_str = j_text.dump();
    PubTopic("jw/robot_arm/lzx",pub_str);
}

void MqttTransition::PublishTask(std::string task_name)
{
    json j_text;
    if(task_name=="initial")
    {
        j_text["name"] = "joint_move";
        j_text["jointPosition"] = {0,0,0,0,0,0};
        std::string pub_str = j_text.dump();
        PubTopic("jw/robot_arm/joint_move",pub_str);
    }
    else
    {
        j_text["name"] = task_name;
        std::string pub_str = j_text.dump();
        PubTopic("jw/robot_arm/tasks",pub_str);
    }

}

void MqttTransition::PublishCallback(const mqtt_client::PublishData::ConstPtr& msg)
{
    std::cout<<msg->task<<std::endl;
    pthis->PublishTask(msg->task);
}

void MqttTransition::PubTopic(std::string topic,std::string pub_str)
{
    if(_client.is_connected())
    {
        _client.publish(topic,pub_str.data(),pub_str.size(),1,false);
    }
}

