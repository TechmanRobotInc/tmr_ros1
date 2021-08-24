#ifdef NO_INCLUDE_DIR
#include "tm_svr_communication.h"
#include "tm_print.h"
#else
#include "tm_driver/tm_svr_communication.h"
#include "tm_driver/tm_print.h"
#endif

#include <functional>

//
// TmSvrCommunication
//

TmSvrCommunication::TmSvrCommunication(const std::string &ip,
	int recv_buf_len, std::condition_variable *cv)
	: TmCommunication(ip.c_str(), 5891, recv_buf_len)
{
	ROS_INFO_STREAM("TM_SVR: TmSvrCommunication");
	if (cv) {
		_cv = cv;
		_has_thread = true;
	}
}

TmSvrCommunication::~TmSvrCommunication()
{
	halt();
}

bool TmSvrCommunication::start_tm_svr(int timeout_ms)
{
	if (socket_description() == 6188)
	{
	    ROS_INFO_STREAM("TM_SVR: start (fake)");
		if (_has_thread) {
			// start thread
			_recv_thread = std::thread(std::bind(&TmSvrCommunication::tm_svr_thread_function, this));
		}
		return true;
	}

	halt();
	ROS_INFO_STREAM("TM_SVR: start");

	bool rb = connect_socket(timeout_ms);
	//if (!rb) return rb; // ? start thread anyway

	if (_has_thread) {
		// start thread
		_recv_thread = std::thread(std::bind(&TmSvrCommunication::tm_svr_thread_function, this));
	}
	return rb;
}

void TmSvrCommunication::halt()
{
	if (socket_description() == 6188)
	{
		ROS_INFO_STREAM("TM_SVR: halt (fake)");
		if (_has_thread) {
			_keep_thread_alive = false;
			if (_recv_thread.joinable()) {
				_recv_thread.join();
			}
		}
		return;
	}
	if (_has_thread) {
		_keep_thread_alive = false;
		if (_recv_thread.joinable()) {
			_recv_thread.join();
		}
	}
	if (is_connected()) {
		ROS_INFO_STREAM("TM_SVR: halt");
		close_socket();
	}
}

TmCommRC TmSvrCommunication::send_content(const std::string &id, TmSvrData::Mode mode, const std::string &content)
{
	std::string cntt = content;
	TmSvrData cmd{ id, mode, cntt.data(), cntt.size(), TmSvrData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}

TmCommRC TmSvrCommunication::send_content_str(const std::string &id, const std::string &content)
{
	std::string cntt = content;
	TmSvrData cmd{ id, TmSvrData::Mode::STRING, cntt.data(), cntt.size(), TmSvrData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}

TmCommRC TmSvrCommunication::send_stick_play()
{
	return send_content_str("Play", "Stick_PlayPause=1");
}

void TmSvrCommunication::tm_svr_thread_function()
{
	ROS_INFO_STREAM("TM_SVR: thread begin");
	_keep_thread_alive = true;
	while (_keep_thread_alive) {
		bool reconnect = false;
		if (!recv_init()) {
			ROS_INFO_STREAM("TM_SVR: is not connected");
		}
		while (_keep_thread_alive && is_connected() && !reconnect) {
			TmCommRC rc = tmsvr_function();
			_updated = true;
			_cv->notify_all();

			switch (rc) {
			case TmCommRC::ERR:
			case TmCommRC::NOTREADY:
			case TmCommRC::NOTCONNECT:
			case TmCommRC::TIMEOUT:
				ROS_INFO_STREAM("TM_SVR: rc=" << int(rc));
				reconnect = true;
				break;
			default: break;
			}
		}
		close_socket();
		reconnect_function();
	}
	close_socket();
	ROS_INFO_STREAM("TM_SVR: thread end");
}

void TmSvrCommunication::reconnect_function()
{
	if (!_keep_thread_alive) return;
	if (_reconnect_timeval_ms <= 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	ROS_INFO_STREAM("TM_SVR: Reconnecting.. ");
	int cnt = 0;
	while (_keep_thread_alive && cnt < _reconnect_timeval_ms) {
		if (cnt % 500 == 0) {
			ROS_DEBUG_STREAM(0.001 * (_reconnect_timeval_ms - cnt) << " sec...");
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		++cnt;
	}
	if (_keep_thread_alive && _reconnect_timeval_ms >= 0) {
		ROS_INFO_STREAM("0 sec\nTM_SVR: connect(" << (int)_reconnect_timeout_ms << "ms)...");
		connect_socket(_reconnect_timeout_ms);
	}
}

TmCommRC TmSvrCommunication::tmsvr_function()
{
	TmCommRC rc;
	int n;
	rc = recv_spin_once(1000, &n);
	if (rc != TmCommRC::OK) {
		return rc;
	}
	std::vector<TmPacket> &pack_vec = packet_list();

	for (auto &pack : pack_vec) {
		if (pack.type == TmPacket::Header::CPERR) {
			tmSvrErrData.set_CPError(pack.data.data(), pack.data.size());
			ROS_ERROR("TM_SVR: CPERR %s",tmSvrErrData.error_code_str().c_str());
		}
		else if (pack.type == TmPacket::Header::TMSVR) {
			
			tmSvrErrData.error_code(TmCPError::Code::Ok);

			TmSvrData::build_TmSvrData(data, pack.data.data(), pack.data.size(), TmSvrData::SrcType::Shallow);
			
			if (data.is_valid()) {
				switch (data.mode()) {
				case TmSvrData::Mode::RESPONSE:
					ROS_INFO_STREAM("TM_SVR: RESPONSE (" << data.transaction_id() << "): [" <<
					(int)(data.error_code()) << "]: " << std::string(data.content(), data.content_len()));
					break;
				case TmSvrData::Mode::BINARY:
					state.mtx_deserialize(data.content(), data.content_len());
					break;
				case TmSvrData::Mode::READ_STRING:
					ROS_INFO_STREAM("TM_SVR: READ_STRING (" << data.transaction_id() << "): " <<
						std::string(data.content(), data.content_len()));
					break;
				default:
					ROS_ERROR_STREAM("TM_SVR: (" << data.transaction_id() << "): invalid mode (" << (int)(data.mode()) << ")");
					break;
				}
			}
			else {
				ROS_ERROR_STREAM("TM_SVR: invalid data");
			}
		}
		else {
			ROS_ERROR_STREAM("TM_SVR: invalid header");
		}
	}
	return rc;
}
