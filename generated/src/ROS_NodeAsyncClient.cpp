
// AUTO GENERATED by vnxcppcodegen

#include <neo_usboard_v2/package.hxx>
#include <neo_usboard_v2/ROS_NodeAsyncClient.hxx>
#include <pilot/usboard/USBoardConfig.hxx>
#include <pilot/usboard/USBoardData.hxx>
#include <vnx/Module.h>
#include <vnx/ModuleInterface_vnx_close.hxx>
#include <vnx/ModuleInterface_vnx_close_return.hxx>
#include <vnx/ModuleInterface_vnx_get_config.hxx>
#include <vnx/ModuleInterface_vnx_get_config_object.hxx>
#include <vnx/ModuleInterface_vnx_get_config_object_return.hxx>
#include <vnx/ModuleInterface_vnx_get_config_return.hxx>
#include <vnx/ModuleInterface_vnx_get_type_code.hxx>
#include <vnx/ModuleInterface_vnx_get_type_code_return.hxx>
#include <vnx/ModuleInterface_vnx_restart.hxx>
#include <vnx/ModuleInterface_vnx_restart_return.hxx>
#include <vnx/ModuleInterface_vnx_set_config.hxx>
#include <vnx/ModuleInterface_vnx_set_config_object.hxx>
#include <vnx/ModuleInterface_vnx_set_config_object_return.hxx>
#include <vnx/ModuleInterface_vnx_set_config_return.hxx>
#include <vnx/TopicPtr.hpp>

#include <vnx/vnx.h>


namespace neo_usboard_v2 {

ROS_NodeAsyncClient::ROS_NodeAsyncClient(const std::string& service_name)
	:	AsyncClient::AsyncClient(vnx::Hash64(service_name))
{
}

ROS_NodeAsyncClient::ROS_NodeAsyncClient(vnx::Hash64 service_addr)
	:	AsyncClient::AsyncClient(service_addr)
{
}

uint64_t ROS_NodeAsyncClient::vnx_get_config_object(const std::function<void(const ::vnx::Object&)>& _callback, const std::function<void(const vnx::exception&)>& _error_callback) {
	auto _method = ::vnx::ModuleInterface_vnx_get_config_object::create();
	const auto _request_id = ++vnx_next_id;
	{
		std::lock_guard<std::mutex> _lock(vnx_mutex);
		vnx_queue_vnx_get_config_object[_request_id] = std::make_pair(_callback, _error_callback);
		vnx_num_pending++;
	}
	vnx_request(_method, _request_id);
	return _request_id;
}

uint64_t ROS_NodeAsyncClient::vnx_get_config(const std::string& name, const std::function<void(const ::vnx::Variant&)>& _callback, const std::function<void(const vnx::exception&)>& _error_callback) {
	auto _method = ::vnx::ModuleInterface_vnx_get_config::create();
	_method->name = name;
	const auto _request_id = ++vnx_next_id;
	{
		std::lock_guard<std::mutex> _lock(vnx_mutex);
		vnx_queue_vnx_get_config[_request_id] = std::make_pair(_callback, _error_callback);
		vnx_num_pending++;
	}
	vnx_request(_method, _request_id);
	return _request_id;
}

uint64_t ROS_NodeAsyncClient::vnx_set_config_object(const ::vnx::Object& config, const std::function<void()>& _callback, const std::function<void(const vnx::exception&)>& _error_callback) {
	auto _method = ::vnx::ModuleInterface_vnx_set_config_object::create();
	_method->config = config;
	const auto _request_id = ++vnx_next_id;
	{
		std::lock_guard<std::mutex> _lock(vnx_mutex);
		vnx_queue_vnx_set_config_object[_request_id] = std::make_pair(_callback, _error_callback);
		vnx_num_pending++;
	}
	vnx_request(_method, _request_id);
	return _request_id;
}

uint64_t ROS_NodeAsyncClient::vnx_set_config(const std::string& name, const ::vnx::Variant& value, const std::function<void()>& _callback, const std::function<void(const vnx::exception&)>& _error_callback) {
	auto _method = ::vnx::ModuleInterface_vnx_set_config::create();
	_method->name = name;
	_method->value = value;
	const auto _request_id = ++vnx_next_id;
	{
		std::lock_guard<std::mutex> _lock(vnx_mutex);
		vnx_queue_vnx_set_config[_request_id] = std::make_pair(_callback, _error_callback);
		vnx_num_pending++;
	}
	vnx_request(_method, _request_id);
	return _request_id;
}

uint64_t ROS_NodeAsyncClient::vnx_get_type_code(const std::function<void(const ::vnx::TypeCode&)>& _callback, const std::function<void(const vnx::exception&)>& _error_callback) {
	auto _method = ::vnx::ModuleInterface_vnx_get_type_code::create();
	const auto _request_id = ++vnx_next_id;
	{
		std::lock_guard<std::mutex> _lock(vnx_mutex);
		vnx_queue_vnx_get_type_code[_request_id] = std::make_pair(_callback, _error_callback);
		vnx_num_pending++;
	}
	vnx_request(_method, _request_id);
	return _request_id;
}

uint64_t ROS_NodeAsyncClient::vnx_restart(const std::function<void()>& _callback, const std::function<void(const vnx::exception&)>& _error_callback) {
	auto _method = ::vnx::ModuleInterface_vnx_restart::create();
	const auto _request_id = ++vnx_next_id;
	{
		std::lock_guard<std::mutex> _lock(vnx_mutex);
		vnx_queue_vnx_restart[_request_id] = std::make_pair(_callback, _error_callback);
		vnx_num_pending++;
	}
	vnx_request(_method, _request_id);
	return _request_id;
}

uint64_t ROS_NodeAsyncClient::vnx_close(const std::function<void()>& _callback, const std::function<void(const vnx::exception&)>& _error_callback) {
	auto _method = ::vnx::ModuleInterface_vnx_close::create();
	const auto _request_id = ++vnx_next_id;
	{
		std::lock_guard<std::mutex> _lock(vnx_mutex);
		vnx_queue_vnx_close[_request_id] = std::make_pair(_callback, _error_callback);
		vnx_num_pending++;
	}
	vnx_request(_method, _request_id);
	return _request_id;
}

std::vector<uint64_t> ROS_NodeAsyncClient::vnx_get_pending_ids() const {
	std::lock_guard<std::mutex> _lock(vnx_mutex);
	std::vector<uint64_t> _list;
	for(const auto& entry : vnx_queue_vnx_get_config_object) {
		_list.push_back(entry.first);
	}
	for(const auto& entry : vnx_queue_vnx_get_config) {
		_list.push_back(entry.first);
	}
	for(const auto& entry : vnx_queue_vnx_set_config_object) {
		_list.push_back(entry.first);
	}
	for(const auto& entry : vnx_queue_vnx_set_config) {
		_list.push_back(entry.first);
	}
	for(const auto& entry : vnx_queue_vnx_get_type_code) {
		_list.push_back(entry.first);
	}
	for(const auto& entry : vnx_queue_vnx_restart) {
		_list.push_back(entry.first);
	}
	for(const auto& entry : vnx_queue_vnx_close) {
		_list.push_back(entry.first);
	}
	return _list;
}

void ROS_NodeAsyncClient::vnx_purge_request(uint64_t _request_id, const vnx::exception& _ex) {
	std::unique_lock<std::mutex> _lock(vnx_mutex);
	{
		const auto _iter = vnx_queue_vnx_get_config_object.find(_request_id);
		if(_iter != vnx_queue_vnx_get_config_object.end()) {
			const auto _callback = std::move(_iter->second.second);
			vnx_queue_vnx_get_config_object.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_ex);
			}
			return;
		}
	}
	{
		const auto _iter = vnx_queue_vnx_get_config.find(_request_id);
		if(_iter != vnx_queue_vnx_get_config.end()) {
			const auto _callback = std::move(_iter->second.second);
			vnx_queue_vnx_get_config.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_ex);
			}
			return;
		}
	}
	{
		const auto _iter = vnx_queue_vnx_set_config_object.find(_request_id);
		if(_iter != vnx_queue_vnx_set_config_object.end()) {
			const auto _callback = std::move(_iter->second.second);
			vnx_queue_vnx_set_config_object.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_ex);
			}
			return;
		}
	}
	{
		const auto _iter = vnx_queue_vnx_set_config.find(_request_id);
		if(_iter != vnx_queue_vnx_set_config.end()) {
			const auto _callback = std::move(_iter->second.second);
			vnx_queue_vnx_set_config.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_ex);
			}
			return;
		}
	}
	{
		const auto _iter = vnx_queue_vnx_get_type_code.find(_request_id);
		if(_iter != vnx_queue_vnx_get_type_code.end()) {
			const auto _callback = std::move(_iter->second.second);
			vnx_queue_vnx_get_type_code.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_ex);
			}
			return;
		}
	}
	{
		const auto _iter = vnx_queue_vnx_restart.find(_request_id);
		if(_iter != vnx_queue_vnx_restart.end()) {
			const auto _callback = std::move(_iter->second.second);
			vnx_queue_vnx_restart.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_ex);
			}
			return;
		}
	}
	{
		const auto _iter = vnx_queue_vnx_close.find(_request_id);
		if(_iter != vnx_queue_vnx_close.end()) {
			const auto _callback = std::move(_iter->second.second);
			vnx_queue_vnx_close.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_ex);
			}
			return;
		}
	}
}

void ROS_NodeAsyncClient::vnx_callback_switch(uint64_t _request_id, std::shared_ptr<const vnx::Value> _value) {
	std::unique_lock<std::mutex> _lock(vnx_mutex);
	const auto _type_hash = _value->get_type_hash();
	if(_type_hash == vnx::Hash64(0xa913f47dc68e4876ull)) {
		auto _result = std::dynamic_pointer_cast<const ::vnx::ModuleInterface_vnx_get_config_object_return>(_value);
		if(!_result) {
			throw std::logic_error("ROS_NodeAsyncClient: !_result");
		}
		const auto _iter = vnx_queue_vnx_get_config_object.find(_request_id);
		if(_iter != vnx_queue_vnx_get_config_object.end()) {
			const auto _callback = std::move(_iter->second.first);
			vnx_queue_vnx_get_config_object.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_result->_ret_0);
			}
		} else {
			throw std::runtime_error("ROS_NodeAsyncClient: received unknown return request_id");
		}
	}
	else if(_type_hash == vnx::Hash64(0xe5b6c635f30b18a1ull)) {
		auto _result = std::dynamic_pointer_cast<const ::vnx::ModuleInterface_vnx_get_config_return>(_value);
		if(!_result) {
			throw std::logic_error("ROS_NodeAsyncClient: !_result");
		}
		const auto _iter = vnx_queue_vnx_get_config.find(_request_id);
		if(_iter != vnx_queue_vnx_get_config.end()) {
			const auto _callback = std::move(_iter->second.first);
			vnx_queue_vnx_get_config.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_result->_ret_0);
			}
		} else {
			throw std::runtime_error("ROS_NodeAsyncClient: received unknown return request_id");
		}
	}
	else if(_type_hash == vnx::Hash64(0xdd5ede96590e3d28ull)) {
		const auto _iter = vnx_queue_vnx_set_config_object.find(_request_id);
		if(_iter != vnx_queue_vnx_set_config_object.end()) {
			const auto _callback = std::move(_iter->second.first);
			vnx_queue_vnx_set_config_object.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback();
			}
		} else {
			throw std::runtime_error("ROS_NodeAsyncClient: received unknown return request_id");
		}
	}
	else if(_type_hash == vnx::Hash64(0x3873b149bdf7814eull)) {
		const auto _iter = vnx_queue_vnx_set_config.find(_request_id);
		if(_iter != vnx_queue_vnx_set_config.end()) {
			const auto _callback = std::move(_iter->second.first);
			vnx_queue_vnx_set_config.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback();
			}
		} else {
			throw std::runtime_error("ROS_NodeAsyncClient: received unknown return request_id");
		}
	}
	else if(_type_hash == vnx::Hash64(0x9f4322ca83b0d1ull)) {
		auto _result = std::dynamic_pointer_cast<const ::vnx::ModuleInterface_vnx_get_type_code_return>(_value);
		if(!_result) {
			throw std::logic_error("ROS_NodeAsyncClient: !_result");
		}
		const auto _iter = vnx_queue_vnx_get_type_code.find(_request_id);
		if(_iter != vnx_queue_vnx_get_type_code.end()) {
			const auto _callback = std::move(_iter->second.first);
			vnx_queue_vnx_get_type_code.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback(_result->_ret_0);
			}
		} else {
			throw std::runtime_error("ROS_NodeAsyncClient: received unknown return request_id");
		}
	}
	else if(_type_hash == vnx::Hash64(0x2133a6eee0102018ull)) {
		const auto _iter = vnx_queue_vnx_restart.find(_request_id);
		if(_iter != vnx_queue_vnx_restart.end()) {
			const auto _callback = std::move(_iter->second.first);
			vnx_queue_vnx_restart.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback();
			}
		} else {
			throw std::runtime_error("ROS_NodeAsyncClient: received unknown return request_id");
		}
	}
	else if(_type_hash == vnx::Hash64(0x88dc702251f03a54ull)) {
		const auto _iter = vnx_queue_vnx_close.find(_request_id);
		if(_iter != vnx_queue_vnx_close.end()) {
			const auto _callback = std::move(_iter->second.first);
			vnx_queue_vnx_close.erase(_iter);
			vnx_num_pending--;
			_lock.unlock();
			if(_callback) {
				_callback();
			}
		} else {
			throw std::runtime_error("ROS_NodeAsyncClient: received unknown return request_id");
		}
	}
	else {
		throw std::runtime_error("ROS_NodeAsyncClient: received unknown return type");
	}
}


} // namespace neo_usboard_v2
