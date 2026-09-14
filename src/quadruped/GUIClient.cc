#include "quadruped/GUIClient.hh"

#include "rtclient/WriteASCII.hh"
#include "rtclient/WriteRaw.hh"
#include "rtclient/WriteML.hh"

namespace loggui {

GUIClient::GUIClient() {}

GUIClient::~GUIClient() {
    disconnect();
    if (_writer) {
        delete _writer;
        _writer = nullptr;
    }
}

bool GUIClient::connect(rtcore::LogServer &logserver) {
    disconnect();
    _client = new rtclient::LogClient("localhost", logserver.getPort(), logserver.getChannel());
    if (!_client) {
        log("Failed to allocate LogClient");
        return false;
    }
    log("LogClient created for localhost:" + std::to_string(logserver.getPort()));
    return true;
}

void GUIClient::disconnect() {
    if (_client) {
        delete _client;
        _client = nullptr;
    }
}

void GUIClient::log(const std::string &msg) {
    if (_log_fn) _log_fn(msg);
}

void GUIClient::configureFromToml(const rtcore::ConfigTable &config) {
    rtcore::ConfigTable log_config;
    bool has_log = config.getTable("log", log_config);
    if (!has_log) {
        log("No [log] section found in config; using defaults");
        return;
    }

    _loggingEnabled = log_config.getBool("enable", false);
    bool gui_control = log_config.getBool("gui_control", false);
    _autoStart = !gui_control;

    _logstart = log_config.getDouble("start", 0.0);
    _logfile = log_config.getString("file_name", "supervisor.log");
    _logperiod = static_cast<unsigned int>(log_config.getInt("period", 1));
    _logformat = log_config.getString("file_format", "ascii");
    if (_logformat != "ascii" && _logformat != "raw" && _logformat != "matlab") {
        log("Unknown log format '" + _logformat + "', using 'ascii'");
        _logformat = "ascii";
    }

    _logvars.clear();
    rtcore::ConfigArray default_vars;
    if (log_config.getArray("vars", default_vars)) {
        for (int i = 0; i < default_vars.size(); i++) {
            std::string varname = default_vars.getStringAt(i);
            if (!varname.empty()) {
                _logvars.push_back(varname);
            } else {
                log("Empty variable name at index " + std::to_string(i) + " in config");
            }
        }
    }

    log("Config: mode=" + std::string(_autoStart ? "autonomous" : "interactive") +
        ", " + std::to_string(_logvars.size()) + " default var(s)");
}

bool GUIClient::registerVars(rtclient::LogTask **task_out) {
    if (task_out) *task_out = nullptr;

    if (!_client) {
        log("registerVars: not connected");
        return false;
    }
    if (_logvars.empty()) {
        log("registerVars: no variables configured");
        return false;
    }
    if (!_client->query()) {
        log("registerVars: query() failed / timed out");
        return false;
    }

    rtclient::LogTask *task = _client->newLog();
    if (!task) {
        log("registerVars: newLog() failed");
        return false;
    }

    for (const auto &varname : _logvars) {
        if (!task->addVar(varname.c_str())) {
            log("registerVars: failed to add " + varname + " (line too long?)");
            return false;
        }
    }

    log("registerVars: registered " + std::to_string(_logvars.size()) + " variable(s)");
    if (task_out) *task_out = task;
    return true;
}

bool GUIClient::refreshVars(unsigned int msec) {
    if (!_client) {
        log("refreshVars: not connected");
        return false;
    }
    if (!_client->query(msec)) {
        log("refreshVars: query() failed / timed out");
        return false;
    }

    _vars.clear();
    unsigned int n = _client->listVars();
    for (unsigned int i = 0; i < n; i++) {
        rtcore::log_name_t *nm = _client->getNextVar();
        if (!nm) break;
        VarInfo v;
        v.name = std::string(&nm->name[0]);
        v.type = static_cast<rtcore::log_var_type_t>(nm->type);
        v.array_size = nm->array;
        _vars.push_back(v);
    }
    log("Discovered " + std::to_string(_vars.size()) + " variables");
    return true;
}

bool GUIClient::createWriter(rtclient::LogTask *task) {
    if (!task) {
        log("createWriter: null task");
        return false;
    }
    if (_writer) {
        delete _writer;
        _writer = nullptr;
    }

    if (_logformat == "ascii") {
        _writer = new rtclient::WriteASCII(_logfile.c_str(), task->varList(),
                                            "Supervisor local data log");
    } else if (_logformat == "raw") {
        _writer = new rtclient::WriteRaw(_logfile.c_str(), task->varList(),
                                          "Supervisor local data log");
    } else if (_logformat == "matlab") {
        _writer = new rtclient::WriteML(_logfile.c_str(), task->varList(),
                                         "Supervisor local data log");
    }

    if (!_writer) {
        log("createWriter: failed to construct writer for format '" + _logformat + "'");
        return false;
    }
    return true;
}

} // namespace loggui