#ifndef GUIMODULE_HH
#define GUIMODULE_HH

#include "rtcore/Module.hh"
#include "quadruped/GUIClient.hh"

#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Check_Browser.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Output.H>

#include <vector>
#include <string>

#define GUIMODULE_NAME "GUIModule"

class MdlTrot;

/** \brief Module wrapping a non-blocking FLTK window for pre-flight log
    configuration (variable selection, format, output file).

    Runs as an ordinary rtcore::Module: init() connects to LogServer and
    builds the window, update() pumps FLTK's event loop non-blockingly via
    Fl::check() once per scheduled cycle, so it never stalls the rest of
    ModuleManager's updateModules() loop. No dedicated thread is used.

    Connection is automatic (found via findModule(LOGSERVER_NAME, 0)); there
    is no IP/port entry or "Connect" button.
*/
class GUIModule : public rtcore::Module {
public:
    GUIModule();
    ~GUIModule() override;

    void init() override;
    void uninit() override;
    void activate() override;
    void deactivate() override;
    void update() override;
    bool isReady() const { return _ready; }

    // Selections made by the operator, available for other modules (e.g.
    // Supervisor) to read. Not yet wired to anything -- see conversation.
    const std::vector<std::string> &selectedVars() const { return _selectedVars; }
    const std::string &logFormat() const { return _logFormat; }
    const std::string &outputFile() const { return _outputFile; }

private:
    rtcore::LogServer *_logserver = nullptr;
    loggui::GUIClient _client;

    Fl_Double_Window *_window = nullptr;
    Fl_Check_Browser *_varBrowser = nullptr;
    Fl_Choice *_formatChoice = nullptr;
    Fl_Input *_fileInput = nullptr;
    Fl_Button *_refreshBtn = nullptr;
    Fl_Button *_applyBtn = nullptr;
    Fl_Box *_statusBox = nullptr;
    Fl_Button *_browseBtn = nullptr;
    Fl_Output *_yawOutput{nullptr};
    Fl_Output *_vxOutput{nullptr};

    std::vector<std::string> _selectedVars;
    std::string _logFormat = "ascii";
    std::string _outputFile = "./logs/log.dat";
    
    MdlTrot *_trotModule{nullptr};
    
    bool _ready = false;
    void buildWindow();
    void populateVarBrowser();
    void doRefresh();
    void doApply();
    void doBrowse();
    void doFormatChanged();
    void syncMotionValues();

    
    static void formatChoiceCb(Fl_Widget *, void *v);
    static void refreshCb(Fl_Widget *, void *v);
    static void applyCb(Fl_Widget *, void *v);
    static void browseCb(Fl_Widget *, void *v);
};

#endif // GUIMODULE_HH