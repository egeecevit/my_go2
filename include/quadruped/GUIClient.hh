#ifndef GUICLIENT_HH
#define GUICLIENT_HH

#include "rtcore/LogMsg.hh"
#include "rtcore/LogServer.hh"
#include "rtcore/ConfigTable.hh"
#include "rtclient/LogClient.hh"
#include "rtclient/LogWriter.hh"

#include <string>
#include <vector>
#include <functional>

namespace loggui {

struct VarInfo {
    std::string name;
    rtcore::log_var_type_t type;
    int array_size;
};

/// \brief Wraps rtclient::LogClient plus config parsing, shared between
/// Supervisor and the standalone GUI. Owns no threads of its own.
class GUIClient {
public:
    using LogFn = std::function<void(const std::string &)>;

    GUIClient();
    ~GUIClient();

    // Connects to the logging server. Does NOT ping/verify reachability --
    // see connect()'s definition for why. Returns false only on allocation
    // failure (practically never).
    bool connect(rtcore::LogServer &logserver);
    void disconnect();
    bool isConnected() const { return _client != nullptr; }
    rtclient::LogClient *client() const { return _client; }

    // Reads the [log] table from `config` and stores period/file/format/vars/
    // enable/gui_control locally. Safe to call even if [log] is absent (logs
    // a message and leaves defaults in place).
    void configureFromToml(const rtcore::ConfigTable &config);

    // Single attempt at registering the configured log variables. See .cc
    // for full behavior/failure semantics.
    bool registerVars(rtclient::LogTask **task_out);

    // Builds the LogWriter (WriteASCII/WriteRaw/WriteML, chosen by the
    // configured file_format) for the given task, using the configured
    // _logfile as the output path. Returns true and stores the writer
    // (retrievable via writer()) on success. Any previously-held writer is
    // deleted first. Caller must have already obtained `task` via
    // registerVars().
    bool createWriter(rtclient::LogTask *task);
    rtclient::LogWriter *writer() const { return _writer; }

    void setLogSink(LogFn fn) { _log_fn = std::move(fn); }

    bool refreshVars(unsigned int msec = 500);
    const std::vector<VarInfo> &vars() const { return _vars; }

    // Accessors for config-driven state, for callers (Supervisor) that used
    // to keep these as their own members.
    bool loggingEnabled() const { return _loggingEnabled && !_logvars.empty(); }
    bool autoStart() const { return _autoStart; }
    double logStart() const { return _logstart; }
    unsigned int logPeriod() const { return _logperiod; }
    const std::string &logFile() const { return _logfile; }
    const std::string &logFormat() const { return _logformat; }
    const std::vector<std::string> &logVars() const { return _logvars; }
    void setLogVars(const std::vector<std::string> &vars) { _logvars = vars; }
    void setLogFormat(const std::string &fmt) {
        _logformat = (fmt == "ascii" || fmt == "raw" || fmt == "matlab") ? fmt : "ascii";
    }
    void setLogFile(const std::string &file) { _logfile = file; }

private:
    void log(const std::string &msg);

    rtclient::LogClient *_client = nullptr;
    rtclient::LogWriter *_writer = nullptr;

    bool _loggingEnabled = false;
    bool _autoStart = true;   // true = autonomous (no GUI control), false = interactive
    double _logstart = 0.0;
    unsigned int _logperiod = 1;
    std::string _logfile = "supervisor.log";
    std::string _logformat = "ascii";
    std::vector<std::string> _logvars;
    std::vector<VarInfo> _vars;

    LogFn _log_fn;
};

} // namespace loggui

#endif  // GUICLIENT_HH