/*
 * neo_usboard_v2_node.cpp
 *
 *  Created on: Oct 14, 2020
 *      Author: mad
 */

#include <ros/ros.h>

#include <vnx/Process.h>

#include <pilot/base/CAN_Proxy.h>
#include <pilot/base/SerialPort.h>
#include <pilot/usboard/USBoardModule.h>
#include <pilot/usboard/USBoardModuleClient.hxx>

#include <neo_usboard_v2/ROS_NodeBase.hxx>


static const vnx::TopicPtr topic_data = "usboard.data";
static const vnx::TopicPtr topic_config = "usboard.config";
static const vnx::TopicPtr topic_can_frames = "usboard.can_frames";
static const vnx::TopicPtr topic_can_request = "usboard.can_request";
static const vnx::TopicPtr topic_serial_data = "usboard.serial_data";
static const vnx::TopicPtr topic_serial_request = "usboard.serial_request";


class ROS_Node : public neo_usboard_v2::ROS_NodeBase {
public:
	ROS_Node(const std::string& _vnx_name)
		:	ROS_NodeBase(_vnx_name),
			usboard_sync("USBoardModule")
	{
	}

protected:
	void main() override
	{
		subscribe(input_data);
		subscribe(input_config);

		// request USBoardConfig
		usboard_sync.request_config();

		Super::main();

		ros::shutdown();
	}

	void handle(std::shared_ptr<const pilot::usboard::USBoardData> value) override
	{
		// TODO: convert and publish on ROS
	}

	void handle(std::shared_ptr<const pilot::usboard::USBoardConfig> value) override
	{
		if(value->transmit_mode == pilot::usboard::USBoardConfig::TRANSMIT_MODE_REQUEST)
		{
			// enable request timer
			set_timeout_millis(update_interval_ms, std::bind(&ROS_Node::update, this));
		}
		int i = 0;
		for(const auto& sensor : value->sensor_config)
		{
			if(sensor.active) {
				sensor_group_enable[i / 4] = true;		// auto enable group for requests
			}
			i++;
		}
		config = value;

		ROS_INFO_STREAM("Got USBoardConfig: " << value->to_string());
	}

	void update()
	{
		// request sensor data
		usboard_sync.request_data(std::vector<bool>(sensor_group_enable.begin(), sensor_group_enable.end()));

		// also request analog data if enabled
		if(config && config->enable_analog_input) {
			usboard_sync.request_analog_data();
		}
	}

private:
	pilot::usboard::USBoardModuleClient usboard_sync;

	std::shared_ptr<const pilot::usboard::USBoardConfig> config;

};


int main(int argc, char** argv)
{
	// initialize ROS
	ros::init(argc, argv, "neo_usboard_v2_node");

	// initialize VNX
	vnx::init("neo_usboard_v2_node", 0, nullptr);

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	std::string can_device;
	std::string serial_port;
	int can_baud_rate = 0;
	double update_rate = 0;

	nh_private.param<std::string>("can_device", can_device, "");
	nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
	nh_private.param<int>("can_baud_rate", can_baud_rate, 1000000);
	nh_private.param<double>("update_rate", update_rate, 5.);

	if(!can_device.empty())
	{
		vnx::Handle<pilot::base::CAN_Proxy> module = new pilot::base::CAN_Proxy("CAN_Proxy");
		module->device = can_device;
		module->baud_rate = can_baud_rate;
		module->input = topic_can_request;
		module->output = topic_can_frames;
		module.start_detached();
	}
	else if(!serial_port.empty())
	{
		vnx::Handle<pilot::base::SerialPort> module = new pilot::base::SerialPort("SerialPort");
		module->port = serial_port;
		module->baud_rate = 19200;
		module->raw_mode = true;
		module->input = topic_serial_request;
		module->output = topic_serial_data;
		module.start_detached();
	}
	else {
		ROS_WARN("No serial port or CAN device configured!");
	}

	{
		vnx::Handle<pilot::usboard::USBoardModule> module = new pilot::usboard::USBoardModule("USBoardModule");
		module->input_can = topic_can_frames;
		module->input_serial = topic_serial_data;
		module->topic_can_request = topic_can_request;
		module->topic_serial_request = topic_serial_request;
		module->output_data = topic_data;
		module->output_config = topic_config;
		module.start_detached();
	}
	{
		vnx::Handle<ROS_Node> module = new ROS_Node("ROS_Node");
		module->input_data = topic_data;
		module->input_config = topic_config;
		module->update_interval_ms = 1000 / update_rate;
		module.start_detached();
	}

	ros::spin();		// process incoming ROS messages

	vnx::close();		// trigger VNX shutdown and wait for all modules to exit

	return 0;
}


