#pragma once
#include "tm_communication.h"

class TmSctCommunication : public TmCommunication
{
private:
	std::condition_variable *_cv = nullptr;
	std::thread _recv_thread;
	bool _keep_thread_alive = false;
	bool _has_thread = false;

private:
	bool _updated = false;

	int _reconnect_timeout_ms = 1000;
	int _reconnect_timeval_ms = 3000;

public:
	TmCPError err_data{ TmCPError::Code::Ok };
	TmSctData sct_data;

private:
	std::mutex mtx_sct;
	std::string _sct_response;

public:
	explicit TmSctCommunication(const std::string &ip,
		int recv_buf_len, std::condition_variable *cv = nullptr);
	~TmSctCommunication();

	bool start(int timeout_ms = 0);
	void halt();

	void set_reconnect_timeout(int timeout_ms)
	{ _reconnect_timeout_ms = timeout_ms; }
	void set_reconnect_timeval(int timeval_ms)
	{ _reconnect_timeval_ms = timeval_ms; }

	TmCommRC send_script_str(const std::string &id, const std::string &script);
	TmCommRC send_script_exit_cmd();

public:
	TmCPError::Code cperr_code() { return err_data.error_code(); }
	std::string sct_response() { return _sct_response; }

public:
	void mtx_sct_lock() { mtx_sct.lock(); }
	void mtx_sct_unlock() { mtx_sct.unlock(); }
	std::string mtx_sct_response()
	{
		std::string rv;
		mtx_sct_lock();
		rv = _sct_response;
		mtx_sct_unlock();
		return rv;
	}

private:
	void thread_function();
	void reconnect_function();
public:
	TmCommRC tmsct_function();
};
