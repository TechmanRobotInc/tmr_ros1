#ifdef NO_INCLUDE_DIR
#include "tm_sct_communication.h"
#include "tm_print.h"
#else
#include "tm_driver/tm_sct_communication.h"
#include "tm_driver/tm_print.h"
#endif

#include <functional>

//
// TmSctCommunication
//

TmSctCommunication::TmSctCommunication(const std::string &ip,
	int recv_buf_len, bool &isOnListenNode, std::condition_variable *cv )
	: TmCommunication(ip.c_str(), 5890,recv_buf_len) , isOnListenNode(isOnListenNode)
{
	print_info("Listen node communication: TmSctCommunication");
	if (cv) {
		_cv = cv;
		_has_thread = true;
	}
}

TmSctCommunication::~TmSctCommunication()
{
	halt();
}

bool TmSctCommunication::start_tm_sct(int timeout_ms)
{
	halt();
	print_info("Listen node communication: start");

	bool rb = connect_socket("listen node communication",timeout_ms);
	//if (!rb) return rb; // ? start thread anyway

	if (_has_thread) {
		// start thread
		_recv_thread = std::thread(std::bind(&TmSctCommunication::tm_sct_thread_function, this));
	}
	return rb;
}

void TmSctCommunication::halt()
{
	if (_has_thread) {
		_keep_thread_alive = false;
		if (_recv_thread.joinable()) {
			_recv_thread.join();
		}
		_updated = true;
		_cv->notify_all();
	}
	if (is_connected()) {
		print_info("Listen node communication: halt");
		close_socket();
	}
}

void TmSctCommunication::check_script_is_exit(std::string script){
  std::string compareString = "ScriptExit()";
  if(compareString.compare(script) == 0){
    isOnListenNode  = false;
  }
}

TmCommRC TmSctCommunication::send_script_str(const std::string &id, const std::string &script)
{
	check_script_is_exit(script);
	std::string sct = script;
	TmSctData cmd{ id, sct.data(), sct.size(), TmSctData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}

TmCommRC TmSctCommunication::send_script_str_silent(const std::string &id, const std::string &script)
{
	std::string sct = script;
	TmSctData cmd{ id, sct.data(), sct.size(), TmSctData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_silent_all(pack);
}

TmCommRC TmSctCommunication::send_script_exit()
{
	return send_script_str("Exit", "ScriptExit()");
}

TmCommRC TmSctCommunication::send_sta_request(const std::string &subcmd, const std::string &subdata)
{
	std::string data = subdata;
	TmStaData req{ subcmd, data.data(), data.size(), TmStaData::SrcType::Shallow };
	TmPacket pack{ req };
	return send_packet_all(pack);
}

std::string TmSctCommunication::mtx_sct_response(std::string &id)
{
	std::string rs;
	{
		std::lock_guard<std::mutex> lck(mtx_sct);
		id = sct_data.script_id();
		rs = std::string{ sct_data.script(), sct_data.script_len() };
	}
	return rs;
}

std::string TmSctCommunication::mtx_sta_response(std::string &cmd)
{
	std::string rs;
	{
		std::lock_guard<std::mutex> lck(mtx_sta);
		cmd = sta_data.subcmd_str();
		rs = std::string{ sta_data.subdata(), sta_data.subdata_len() };
	}
	return rs;
}

void TmSctCommunication::tm_sct_thread_function()
{
	print_info("Listen node communication: thread begin");
	_keep_thread_alive = true;
	while (_keep_thread_alive) {
		bool reconnect = false;
		if (!recv_init()) {
			print_info("Listen node communication: is not connected");
		}
		while (_keep_thread_alive && is_connected() && !reconnect) {
			TmCommRC rc = tmsct_function();
			{
				std::lock_guard<std::mutex> lck(_mtx);
				_updated = true;
			}
			_cv->notify_all();

			switch (rc) {
			case TmCommRC::ERR:
			case TmCommRC::NOTREADY:
			case TmCommRC::NOTCONNECT:
				print_info("Listen node communication: rc=%d", int(rc));
				reconnect = true;
				break;
			default: break;
			}
		}
		close_socket();
		reconnect_function();
	}
	close_socket();
	print_info("TM_SCT Listen node communication: thread end");
}

void TmSctCommunication::reconnect_function()
{
	if (!_keep_thread_alive) return;
	if (_reconnect_timeval_ms <= 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	print_info("Listen node communication: Reconnecting.. ");
	int cnt = 0;
	while (_keep_thread_alive && cnt < _reconnect_timeval_ms) {
		if (cnt % 1000 == 0) {
			print_debug("%.1f sec...", 0.001 * (_reconnect_timeval_ms - cnt));
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		++cnt;
	}
	if (_keep_thread_alive && _reconnect_timeval_ms >= 0) {
		print_info("0 sec\n Listen node communication: connect(%dms)...", (int)_reconnect_timeout_ms);
		connect_socket("Listen node communication",_reconnect_timeout_ms);
	}
}

TmCommRC TmSctCommunication::tmsct_function()
{
	TmCommRC rc;
	int n;
	rc = recv_spin_once(1000, &n);
	if (rc != TmCommRC::OK) {
		return rc;
	}
	std::vector<TmPacket> &pack_vec = packet_list();

	//TmCPError err_data_tmp;
	TmSctData sct_data_tmp;
	TmStaData sta_data_tmp;

	for (auto &pack : pack_vec) {
		switch (pack.type) {
		case TmPacket::Header::CPERR:
			tmSctErrData.set_CPError(pack.data.data(), pack.data.size());
            print_error("TM_SCT: CPERR %s",tmSctErrData.error_code_str().c_str());
			break;

		case TmPacket::Header::TMSCT:
			//print_info("TM_SCT: TMSCT");
			tmSctErrData.error_code(TmCPError::Code::Ok);

			/*TmSctData::build_TmSctData(sct_data, pack.data.data(), pack.data.size(), TmSctData::SrcType::Shallow);
			{
				std::lock_guard<std::mutex> lck(mtx_sct);
				_sct_res_id = sct_data.script_id();
				_sct_res_script = std::string{ sct_data.script(), sct_data.script_len() };
			}
			if (sct_data.has_error()) {
				print_error("TM_SCT: err: (%s) %s", _sct_res_id.c_str(), _sct_res_script.c_str());
			}
			else {
				print_info("TM_SCT: res: (%s) %s", _sct_res_id.c_str(), _sct_res_script.c_str());
			}*/

			TmSctData::build_TmSctData(sct_data_tmp, pack.data.data(), pack.data.size(), TmSctData::SrcType::Shallow);
			{
				std::lock_guard<std::mutex> lck(mtx_sct);
				TmSctData::build_TmSctData(sct_data, sct_data_tmp, TmSctData::SrcType::Deep);
			}
			if (sct_data.sct_has_error())
			{				
				print_error("TM_SCT: err: (%s): %s", sct_data.script_id().c_str(), sct_data.script());
			}				
			else
			{				
				print_info("TM_SCT: res: (%s): %s", sct_data.script_id().c_str(), sct_data.script());
			}				

			break;

		case TmPacket::Header::TMSTA:
			//print_info("TM_SCT: TMSTA");
			tmSctErrData.error_code(TmCPError::Code::Ok);

			TmStaData::build_TmStaData(sta_data_tmp, pack.data.data(), pack.data.size(), TmStaData::SrcType::Shallow);
			{
				std::lock_guard<std::mutex> lck(mtx_sta);
				TmStaData::build_TmStaData(sta_data, sta_data_tmp, TmStaData::SrcType::Deep);
			}
			print_info("TM_STA: res: (%s): %s", sta_data.subcmd_str().c_str(), sta_data.subdata());

			tmsta_function();
			break;

		default:
			print_error("TM_SCT: invalid header");
			break;
		}
	}
	return rc;
}

void TmSctCommunication::tmsta_function()
{
	switch (sta_data.subcmd()) {
	case 0:
		break;
	case 1:
		break;
	}
}
