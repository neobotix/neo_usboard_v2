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
#include <dynamic_reconfigure/server.h>
#include "neo_usboard_v2/neo_usboard_v2Config.h"


class ROS_Node : public neo_usboard_v2::ROS_NodeBase, public ros::NodeHandle {
public:
	ros::NodeHandle nh_;
	ros::Publisher topicPub_usBoard;
	ros::Publisher topicPub_USRangeSensor[16];

	ROS_Node(const std::string& _vnx_name, ros::NodeHandle *nh)
		:	ROS_NodeBase(_vnx_name),
			usboard_sync("USBoardModule")
	{
		
		nh->param<std::string>("can_device", can_device, "");
		nh->param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
		nh->param<std::string>("topic_path", topic_path, "/usboard_v2");
		nh->param<int>("can_id", can_id, 1024);
		nh->param<int>("can_baud_rate", can_baud_rate, 1000000);
		nh->param<double>("update_rate", update_rate, 5.);
		nh->param<std::vector<bool>>("active_sensors", active_sensors, active_sensors);
		nh->param<std::vector<double>>("warn_distance", warn_distance, warn_distance);
		nh->param<std::vector<double>>("alarm_distance", alarm_distance, alarm_distance);

		nh->param<std::vector<bool>>("enable_transmission", enable_transmission, enable_transmission);
		nh->param<std::vector<int>>("resolution", resolution, resolution);
		nh->param<std::vector<int>>("fire_interval_ms", fire_interval_ms, fire_interval_ms);
		nh->param<std::vector<int>>("sending_sensor", sending_sensor, sending_sensor);
		nh->param<std::vector<bool>>("cross_echo_mode", cross_echo_mode, cross_echo_mode);

		nh->param<double>("low_pass_gain", low_pass_gain, 0.0);
		nh->param<bool>("enable_analog_input", enable_analog_input, false);
		nh->param<bool>("enable_legacy_format", enable_legacy_format, false);
		nh->param<bool>("enable_can_termination", enable_can_termination, false);
		nh->param<bool>("relay_warn_blocked_invert", relay_warn_blocked_invert, false);
		nh->param<bool>("relay_alarm_blocked_invert", relay_alarm_blocked_invert, false);
		nh_ = *nh;
	}

	void set_ros_params(std::shared_ptr<const pilot::usboard::USBoardConfig> value)
	{
		// prevenable_transmission our own changes from triggering lots of param updates
		ros_params_initialized = false;

		serial_number = value->serial_number;
		hardware_version = value->hardware_version;
		can_id = value->can_id;
		can_baud_rate = value->can_baudrate;
		update_rate = value->update_interval_ms;
		
		for (auto i = 0; i < active_sensors.size(); i++) {
			active_sensors[i] = value->sensor_config[i].active;
			warn_distance[i] = value->sensor_config[i].warn_distance;
			alarm_distance[i] = value->sensor_config[i].alarm_distance;
		}

		for (auto i = 0; i < enable_transmission.size(); i++) {
			enable_transmission[i] = value->group_config[i].enable_transmission;
			resolution[i] = value->group_config[i].resolution;
			fire_interval_ms[i] = value->group_config[i].fire_interval_ms;
			sending_sensor[i] = value->group_config[i].sending_sensor;
			cross_echo_mode[i] = value->group_config[i].cross_echo_mode;
		}
		
		low_pass_gain = value->low_pass_gain;
		enable_analog_input = value->enable_analog_input;
		enable_legacy_format = value->enable_legacy_format;
		enable_can_termination = value->enable_can_termination;
		relay_warn_blocked_invert = value->relay_warn_blocked_invert;
		relay_alarm_blocked_invert = value->relay_alarm_blocked_invert;

		nh_.setParam("hardware_version", hardware_version);
		nh_.setParam("serial_number", serial_number);

		nh_.setParam("can_id", can_id);
		nh_.setParam("can_baud_rate", can_baud_rate);
		nh_.setParam("update_rate", update_rate);

		nh_.setParam("active_sensors", active_sensors);
		nh_.setParam("warn_distance", warn_distance);
		nh_.setParam("alarm_distance", alarm_distance);

		nh_.setParam("enable_transmission", enable_transmission);
		nh_.setParam("resolution", resolution);
		nh_.setParam("fire_interval_ms", fire_interval_ms);
		nh_.setParam("sending_sensor", sending_sensor);
		nh_.setParam("cross_echo_mode", cross_echo_mode);

		nh_.setParam("low_pass_gain", low_pass_gain);
		nh_.setParam("enable_analog_input", enable_analog_input);
		nh_.setParam("enable_legacy_format", enable_legacy_format);
		nh_.setParam("enable_can_termination", enable_can_termination);
		nh_.setParam("relay_warn_blocked_invert", relay_warn_blocked_invert);
		nh_.setParam("relay_alarm_blocked_invert", relay_alarm_blocked_invert);

		ros_params_initialized = true;
	}

protected:
	void main() override
	{
		neo_usboard_v2::ROS_NodeBase::subscribe(input_data);
		neo_usboard_v2::ROS_NodeBase::subscribe(input_config);

		set_timer_millis(1000, std::bind(&ROS_Node::request_config, this));

		topicPub_usBoard = nh_.advertise<neo_msgs::USBoardV2>(topic_path + "/measuremenable_transmissions", 1);

    	server = new dynamic_reconfigure::Server<neo_usboard_v2::neo_usboard_v2Config>(nh_);	
		dynamic_reconfigure::Server<neo_usboard_v2::neo_usboard_v2Config>::CallbackType cb=
			[this](neo_usboard_v2::neo_usboard_v2Config& config, uint32_t level){ reconfigureCB(config, level); };
    	server->setCallback(cb);

		Super::main();

		ros::shutdown();
	}

	void handle(std::shared_ptr<const pilot::usboard::USBoardData> value) override
	{
		neo_msgs::USBoardV2 usBoard;
		usBoard.header.stamp = ros::Time::now();
		usBoard.header.frame_id = "USSensors";
		
		for(int i=0; i < 16; i++)
		{
			const bool is_active = config && config->sensor_config[i].active;
			usBoard.active[i] = is_active;

			if(is_active)
			{
				sensor_msgs::Range USRangeMsg;
				USRangeMsg.header.stamp = ros::Time::now(); 		//time
				USRangeMsg.header.frame_id = "usrangesensor"+std::to_string(i);		//string
				USRangeMsg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
				USRangeMsg.field_of_view = 2.3; 		//float32 [rad]
				USRangeMsg.min_range = 0.2; 			//float32 [m]
				USRangeMsg.max_range = 3.0; 			//float32 [m]
				USRangeMsg.range = value->sensor[i]; 	//float32 [m]

				if(topicPub_USRangeSensor[i]) {
					topicPub_USRangeSensor[i].publish(USRangeMsg);
				}
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
				topicPub_USRangeSensor[i] = nh_.advertise<sensor_msgs::Range>(topic_path + "/sensor" + std::to_string(i), 1);
			}
			i++;
		}
		config = value;
               set_ros_params(value);
		ROS_INFO_STREAM("Got USBoardConfig: " << value->to_string());
	}

	void update()
	{
		std::lock_guard<std::mutex> lock(mutex_usboard_sync);

		// request sensor data
		usboard_sync.request_data(std::vector<bool>(sensor_group_enable.begin(), sensor_group_enable.end()));

		// also request analog data if enabled
		if(config && config->enable_analog_input) {
			usboard_sync.request_analog_data();
		}
	}

	void request_config()
	{
		std::lock_guard<std::mutex> lock(mutex_usboard_sync);
		try {
			if(!config) {
				usboard_sync.request_config();
			}
		} catch(const std::exception& ex) {
			ROS_ERROR_STREAM("Failed to get USBoardConfig: " << ex.what());
		}
	}

	void set_pilot_params(std::shared_ptr<pilot::usboard::USBoardConfig> value) const
	{
		value->can_id = can_id;
		value->can_baudrate = can_baud_rate;
		value->update_interval_ms = update_rate;
		
		for (auto i = 0; i < active_sensors.size(); i++) {
			value->sensor_config[i].active = active_sensors[i];
			value->sensor_config[i].warn_distance = warn_distance[i];
			value->sensor_config[i].alarm_distance = alarm_distance[i];
		}

		for (auto i = 0; i < enable_transmission.size(); i++) {
			value->group_config[i].enable_transmission = enable_transmission[i];
			value->group_config[i].resolution = resolution[i];
			value->group_config[i].fire_interval_ms = fire_interval_ms[i];
			value->group_config[i].sending_sensor = sending_sensor[i];
			value->group_config[i].cross_echo_mode = cross_echo_mode[i];
		}
		
		value->low_pass_gain = low_pass_gain;
		value->enable_analog_input = enable_analog_input;
		value->enable_legacy_format = enable_legacy_format;
		value->enable_can_termination = enable_can_termination;
		value->relay_warn_blocked_invert = relay_warn_blocked_invert;
		value->relay_alarm_blocked_invert = relay_alarm_blocked_invert;
	}

	void reconfigureCB(neo_usboard_v2::neo_usboard_v2Config &val, uint32_t level)
	{
		if(ros_params_initialized) {

			active_sensors = {val.active_sensor_0, val.active_sensor_1, val.active_sensor_2, val.active_sensor_3, val.active_sensor_4,
			val.active_sensor_5, val.active_sensor_6, val.active_sensor_7, val.active_sensor_8, val.active_sensor_9, val.active_sensor_10,
			val.active_sensor_11, val.active_sensor_12, val.active_sensor_13, val.active_sensor_14, val.active_sensor_15 };
		
			warn_distance = {val.warn_distance_0, val.warn_distance_1, val.warn_distance_2, val.warn_distance_3, val.warn_distance_4,
				val.warn_distance_5, val.warn_distance_6, val.warn_distance_7, val.warn_distance_8, val.warn_distance_9, val.warn_distance_10,
				val.warn_distance_11, val.warn_distance_12, val.warn_distance_13, val.warn_distance_14, val.warn_distance_15 };
			
			alarm_distance = {val.alarm_distance_0, val.alarm_distance_1, val.alarm_distance_2, val.alarm_distance_3, val.alarm_distance_4,
				val.alarm_distance_5, val.alarm_distance_6, val.alarm_distance_7, val.alarm_distance_8, val.alarm_distance_9, val.alarm_distance_10,
				val.alarm_distance_11, val.alarm_distance_12, val.alarm_distance_13, val.alarm_distance_14, val.alarm_distance_15 };

			enable_transmission = {val.enable_transmission_0, val.enable_transmission_1, val.enable_transmission_2, val.enable_transmission_3};
			fire_interval_ms = {val.fire_interval_ms_0, val.fire_interval_ms_1, val.fire_interval_ms_2, val.fire_interval_ms_3};
			sending_sensor = {val.sending_sensor_0, val.sending_sensor_1, val.sending_sensor_2, val.sending_sensor_3};
			cross_echo_mode = {val.cross_echo_mode_0, val.cross_echo_mode_1, val.cross_echo_mode_2, val.cross_echo_mode_3};

			can_id = val.can_id;
			can_baud_rate = val.can_baud_rate;
			update_rate = val.update_rate;

			low_pass_gain = val.low_pass_gain;
			enable_analog_input = val.enable_analog_input;
			enable_legacy_format = val.enable_legacy_format;
			enable_can_termination = val.enable_can_termination;
			relay_warn_blocked_invert = val.relay_warn_blocked_invert;
			relay_alarm_blocked_invert = val.relay_alarm_blocked_invert;

			std::shared_ptr<pilot::usboard::USBoardConfig> new_config = vnx::clone(config);
			set_pilot_params(new_config);
			{
				std::lock_guard<std::mutex> lock(mutex_usboard_sync);
				std::cout << "Updating parameter config: " << new_config->to_string() << std::endl;
				try{
					usboard_sync.send_config(new_config);
					config = new_config;
				}catch(const std::exception &err){
					ROS_WARN("Sending USBoard config failed with: %s", err.what());
				}
			}
		}
		
	}

private:
	pilot::usboard::USBoardModuleClient usboard_sync;

	std::shared_ptr<const pilot::usboard::USBoardConfig> config;
	dynamic_reconfigure::Server<neo_usboard_v2::neo_usboard_v2Config> *server;
	std::mutex mutex_usboard_sync;

public:
	std::string can_device = "None";
	std::string serial_port = "/dev/ttyUSB0";
	std::string topic_path = "/usboard_v2";

	int serial_number;
	int hardware_version;

	int can_id = 0;
	int can_baud_rate = 0;
	double update_rate = 0.0;
	bool ros_params_initialized = false;
	
	// Our new generation supports 16 sensors. Therefore the size of the vector won't go beyond 16

	std::vector<bool> active_sensors = std::vector<bool>(16, false);
	std::vector<double> warn_distance = std::vector<double>(16, 0.0);
	std::vector<double> alarm_distance = std::vector<double>(16, 0.0);

	// There are 4 sensor groups, following params deals with that

	std::vector<bool> enable_transmission = std::vector<bool>(4, false);
	std::vector<int> resolution = std::vector<int>(4, 0);
	std::vector<int> fire_interval_ms = std::vector<int>(4, 0);
	std::vector<int> sending_sensor = std::vector<int>(4, 0);
	std::vector<bool> cross_echo_mode = std::vector<bool>(4, false);

	double low_pass_gain = 0.0;
	bool enable_analog_input = false;
	bool enable_legacy_format = false;
	bool enable_can_termination = false;
	bool relay_warn_blocked_invert = false;
	bool relay_alarm_blocked_invert = false;

};


int main(int argc, char** argv)
{
	// initialize ROS
	ros::init(argc, argv, "neo_usboard_v2_node");

	// initialize VNX
	vnx::init("neo_usboard_v2_node", 0, nullptr);

	ros::NodeHandle nh;

	auto ros_node = new ROS_Node("neo_usboard_v2_node", &nh);

	if(ros_node->can_device != "None")
	{
		vnx::Handle<pilot::base::CAN_Proxy> module = new pilot::base::CAN_Proxy("CAN_Proxy");
		module->device = ros_node->can_device;
		module->baud_rate = ros_node->can_baud_rate;
		module->input = neo_usboard_v2::can_request;
		module->output = neo_usboard_v2::can_frames;
		module.start_detached();
		std::cout<<"Started " << module.get_name() << " on " << ros_node->can_device <<std::endl;
	}
	else if(!ros_node->serial_port.empty())
	{
		vnx::Handle<pilot::base::SerialPort> module = new pilot::base::SerialPort("SerialPort");
		module->port = ros_node->serial_port;
		module->baud_rate = 19200;
		module->raw_mode = true;
		module->input = neo_usboard_v2::serial_request;
		module->output = neo_usboard_v2::serial_data;
		module.start_detached();
		std::cout<<"Started " << module.get_name() << " on " << ros_node->serial_port <<std::endl;
	}
	else {
		std::cout<<"No serial port or CAN device configured!"<<std::endl;
	}

	{
		vnx::Handle<pilot::usboard::USBoardModule> module = new pilot::usboard::USBoardModule("USBoardModule");
		module->input_can = neo_usboard_v2::can_frames;
		module->input_serial = neo_usboard_v2::serial_data;
		module->topic_can_request = neo_usboard_v2::can_request;
		module->topic_serial_request = neo_usboard_v2::serial_request;
		module->output_data = neo_usboard_v2::data;
		module->output_config = neo_usboard_v2::config;
		module->can_id = ros_node->can_id;
		module.start_detached();
	}
	{
		vnx::Handle<ROS_Node> module = ros_node;
		module->input_data = neo_usboard_v2::data;
		module->input_config = neo_usboard_v2::config;
		module->update_interval_ms = 1000 / ros_node->update_rate;
		module->topic_path = ros_node->topic_path;
		module.start_detached();
	}

	ros::spin();		// process incoming ROS messages

	vnx::close();		// trigger VNX shutdown and wait for all modules to exit

	return 0;
}