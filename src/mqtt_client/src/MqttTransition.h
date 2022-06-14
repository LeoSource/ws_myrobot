#pragma once

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <atomic>
#include "mqtt/async_client.h"
#include "json.hpp"
#include "mqtt_client/PublishData.h"
#include "mqtt_client/SubscribeData.h"

using nlohmann::json;

const std::string SERVER_ADDRESS("tcp://192.168.100.11:1883");
const std::string CLIENT_ID("XArm-Visualisation");
const std::string TOPIC("jw/robot_arm/data_send");
const int QOS = 1;
const int N_RETRY_ATTEMPTS = 50;

class action_listener : public virtual mqtt::iaction_listener
{
	std::string name_;

	void on_failure(const mqtt::token& tok) override {
		std::cout << name_ << " failure";
		if (tok.get_message_id() != 0)
			std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
		std::cout << std::endl;
	}

	void on_success(const mqtt::token& tok) override {
		std::cout << name_ << " success";
		if (tok.get_message_id() != 0)
			std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
		auto top = tok.get_topics();
		if (top && !top->empty())
			std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
		std::cout << std::endl;
	}

public:
	action_listener(const std::string& name) : name_(name) {}
};


class callback : public virtual mqtt::callback,
					public virtual mqtt::iaction_listener
{
	// Counter for the number of connection retries
	int nretry_;
	// The MQTT client
	mqtt::async_client& cli_;
	// Options to use if we need to reconnect
	mqtt::connect_options& connOpts_;
	// An action listener to display the result of actions.
	action_listener subListener_;

	// This deomonstrates manually reconnecting to the broker by calling
	// connect() again. This is a possibility for an application that keeps
	// a copy of it's original connect_options, or if the app wants to
	// reconnect with different options.
	// Another way this can be done manually, if using the same options, is
	// to just call the async_client::reconnect() method.
	void reconnect() {
		std::this_thread::sleep_for(std::chrono::milliseconds(2500));
		try {
			cli_.connect(connOpts_, nullptr, *this);
		}
		catch (const mqtt::exception& exc) {
			std::cerr << "Error: " << exc.what() << std::endl;
			exit(1);
		}
	}

	// Re-connection failure
	void on_failure(const mqtt::token& tok) override {
		std::cout << "Connection attempt failed" << std::endl;
		if (++nretry_ > N_RETRY_ATTEMPTS)
			exit(1);
		reconnect();
	}

	// (Re)connection success
	// Either this or connected() can be used for callbacks.
	void on_success(const mqtt::token& tok) override {}

	// (Re)connection success
	void connected(const std::string& cause) override {
		std::cout << "\nConnection success" << std::endl;
		std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
			<< "\tfor client " << CLIENT_ID
			<< " using QoS" << QOS << "\n"
			<< "\nPress Q<Enter> to quit\n" << std::endl;

		cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
	}

	// Callback for when the connection is lost.
	// This will initiate the attempt to manually reconnect.
	void connection_lost(const std::string& cause) override {
		std::cout << "\nConnection lost" << std::endl;
		if (!cause.empty())
			std::cout << "\tcause: " << cause << std::endl;

		std::cout << "Reconnecting..." << std::endl;
		nretry_ = 0;
		reconnect();
	}

	// Callback for when a message arrives.
	void message_arrived(mqtt::const_message_ptr msg) override {
		// std::cout << "Message arrived" << std::endl;
		// std::cout << "topic: '" << msg->get_topic() << "'" << std::endl;
        auto json_str = json::parse(msg->to_string());
        json_str.at("joint_actual_position").get_to(_jpos);
        // std::cout<<jp[0]<<std::endl;
        // std::cout<<jp[1]<<std::endl;
        // std::cout<<jp[2]<<std::endl;
        // std::cout<<jp[3]<<std::endl;
        // std::cout<<jp[4]<<std::endl;
        // std::cout<<jp[5]<<std::endl;
		// std::cout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;
	}

	void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
	callback(mqtt::async_client& cli, mqtt::connect_options& connOpts)
				: nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}

public:
    std::vector<double> _jpos;
};


class MqttTransition
{
private:
    mqtt::async_client& _client;
    mqtt::connect_options& _connOpts;
    callback& _cb;

	static MqttTransition* pthis;

public:
    MqttTransition(mqtt::async_client& clinet,mqtt::connect_options connOpts,callback& cb)
                :_client(clinet),_connOpts(connOpts),_cb(cb) {pthis = this;}

    int Connect();

    int DisConnect();

	bool IsConnected();

	void GetJointPos(std::vector<double>& jpos);

	void StartRecordData();

	void StopRecordData();

	void PublishTask(std::string task_name);

	void static PublishCallback(const mqtt_client::PublishData::ConstPtr& msg);

private:
	void PubTopic(std::string topic,std::string pub_str);


};