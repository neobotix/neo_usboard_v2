
// AUTO GENERATED by vnxcppcodegen

#ifndef INCLUDE_neo_usboard_v2_ROS_Node_CLIENT_HXX_
#define INCLUDE_neo_usboard_v2_ROS_Node_CLIENT_HXX_

#include <vnx/Client.h>
#include <pilot/usboard/USBoardConfig.hxx>
#include <pilot/usboard/USBoardData.hxx>
#include <vnx/Module.h>
#include <vnx/TopicPtr.hpp>


namespace neo_usboard_v2 {

class ROS_NodeClient : public vnx::Client {
public:
	ROS_NodeClient(const std::string& service_name);
	
	ROS_NodeClient(vnx::Hash64 service_addr);
	
	::vnx::Object vnx_get_config_object();
	
	::vnx::Variant vnx_get_config(const std::string& name = "");
	
	void vnx_set_config_object(const ::vnx::Object& config = ::vnx::Object());
	
	void vnx_set_config_object_async(const ::vnx::Object& config = ::vnx::Object());
	
	void vnx_set_config(const std::string& name = "", const ::vnx::Variant& value = ::vnx::Variant());
	
	void vnx_set_config_async(const std::string& name = "", const ::vnx::Variant& value = ::vnx::Variant());
	
	::vnx::TypeCode vnx_get_type_code();
	
	std::shared_ptr<const ::vnx::ModuleInfo> vnx_get_module_info();
	
	void vnx_restart();
	
	void vnx_restart_async();
	
	void vnx_stop();
	
	void vnx_stop_async();
	
	vnx::bool_t vnx_self_test();
	
};


} // namespace neo_usboard_v2

#endif // INCLUDE_neo_usboard_v2_ROS_Node_CLIENT_HXX_
