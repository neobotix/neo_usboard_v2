/*
 * neo_usboard_v2_node.cpp
 *
 *  Created on: Oct 14, 2020
 *      Author: mad
 */

#include <ros/ros.h>

#include <vnx/vnx.h>
#include <pilot/base/CAN_Proxy.h>
#include <pilot/base/SerialPort.h>
#include <pilot/usboard/USBoardModule.h>
#include <pilot/usboard/USBoardModuleClient.hxx>
#include <neo_msgs/USBoardV2.h>
#include <sensor_msgs/Range.h>
#include <neo_usboard_v2/package.hxx>
#include <neo_usboard_v2/ROS_NodeBase.hxx>


class ROS_Node : public neo_usboard_v2::ROS_NodeBase {
public:
	ros::NodeHandle nh;
	ros::Publisher topicPub_usBoard = nh.advertise<neo_msgs::USBoardV2>("/usboard_v2/measurements",1);
	ros::Publisher topicPub_USRangeSensor[16];
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
		neo_msgs::USBoardV2 usBoard;
		usBoard.header.stamp = ros::Time::now();
		usBoard.header.frame_id = "USSensors";
		
		for(int i=0; i < 16; i++)
		{
			sensor_msgs::Range USRangeMsg;
			usBoard.active[i] = config->sensor_config[i].active;
			USRangeMsg.header.stamp = ros::Time::now(); 		//time
			USRangeMsg.header.frame_id = "usrangesensor"+std::to_string(i);		//string
			USRangeMsg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
			USRangeMsg.field_of_view = 2.3; 		//float32 [rad]
			USRangeMsg.min_range = 0.2; 			//float32 [m]
			USRangeMsg.max_range = 3.0; 			//float32 [m]
			USRangeMsg.range = value->sensor[i]; 	//float32 [m]

			//publish data for first USrange sensor
			if(config->sensor_config[i].active)
			{
				topicPub_USRangeSensor[i].publish(USRangeMsg);
			}
			usBoard.sensor[i] = value->sensor[i];
		}

		for(int i = 0; i < 4; i++)
		{
			usBoard.analog[i] = value->analog_input[i];
		}
		topicPub_usBoard.publish(usBoard);
	}

	void handle(std::shared_ptr<const pilot::usboard::USBoardConfig> value) override
	{
		if(value->transmit_mode == pilot::usboard::USBoardConfig::TRANSMIT_MODE_REQUEST)
		{
			// enable request timer
			set_timer_millis(update_interval_ms, std::bind(&ROS_Node::update, this));
		}
		int i = 0;
		for(const auto& sensor : value->sensor_config)
		{
			if(sensor.active) {
				sensor_group_enable[i / 4] = true;		// auto enable group for requests
			}
			topicPub_USRangeSensor[i] = nh.advertise<sensor_msgs::Range>("/usboard_v2/sensor"+std::to_string(i),1);
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
		module->input = neo_usboard_v2::can_request;
		module->output = neo_usboard_v2::can_frames;
		module.start_detached();
	}
	else if(!serial_port.empty())
	{
		vnx::Handle<pilot::base::SerialPort> module = new pilot::base::SerialPort("SerialPort");
		module->port = serial_port;
		module->baud_rate = 19200;
		module->raw_mode = true;
		module->input = neo_usboard_v2::serial_request;
		module->output = neo_usboard_v2::serial_data;
		module.start_detached();
	}
	else {
		ROS_WARN("No serial port or CAN device configured!");
	}

	{
		vnx::Handle<pilot::usboard::USBoardModule> module = new pilot::usboard::USBoardModule("USBoardModule");
		module->input_can = neo_usboard_v2::can_frames;
		module->input_serial = neo_usboard_v2::serial_data;
		module->topic_can_request = neo_usboard_v2::can_request;
		module->topic_serial_request = neo_usboard_v2::serial_request;
		module->output_data = neo_usboard_v2::data;
		module->output_config = neo_usboard_v2::config;
		module.start_detached();
	}
	{
		vnx::Handle<ROS_Node> module = new ROS_Node("ROS_Node");
		module->input_data = neo_usboard_v2::data;
		module->input_config = neo_usboard_v2::config;
		module->update_interval_ms = 1000 / update_rate;
		module.start_detached();
	}

	ros::spin();		// process incoming ROS messages

	vnx::close();		// trigger VNX shutdown and wait for all modules to exit

	return 0;
}