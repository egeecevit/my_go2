#include "quadruped/GUIModule.hh"
#include "quadruped/MdlTrot.hh"
#include "rtcore/ModuleManager.hh"

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Check_Browser.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Native_File_Chooser.H>
#include <filesystem>

// Custom modern color palette
namespace UITheme {
    constexpr Fl_Color rgb(uchar r, uchar g, uchar b) {
        return ((Fl_Color)r << 24) | ((Fl_Color)g << 16) | ((Fl_Color)b << 8);
    }

    constexpr Fl_Color BgColor       = rgb(30, 32, 38);     // Deep slate background
    constexpr Fl_Color SurfaceColor  = rgb(42, 45, 52);     // Card / container surface
    constexpr Fl_Color InputBg       = rgb(20, 21, 25);     // Slightly darker listbox background
    constexpr Fl_Color TextPrimary   = FL_WHITE;            // Crisp pure white for primary elements
    constexpr Fl_Color TextMuted     = rgb(185, 190, 202);  // Brightened secondary label text
    constexpr Fl_Color BtnSecondary  = rgb(58, 63, 74);     // Neutral button background
    constexpr Fl_Color BtnPrimary    = rgb(38, 140, 120);   // Accent green/teal
    constexpr Fl_Color AccentHover   = rgb(48, 165, 140);
    constexpr Fl_Color BorderColor   = rgb(60, 64, 75);
}

GUIModule::GUIModule() : Module(GUIMODULE_NAME, 0, MULTI_USER) {}

GUIModule::~GUIModule() {}

void GUIModule::init() {
    _logserver = (rtcore::LogServer *)_mgr->findModule(LOGSERVER_NAME, 0);
    if (!_logserver) {
        _mgr->warning("GUIModule", "LogServer module not found -- GUI will have no data");
    } else if (!_client.connect(*_logserver)) {
        _mgr->warning("GUIModule", "Failed to create LogClient for GUI");
    }
    _trotModule = (MdlTrot *)_mgr->findModule(TROTMODULE_NAME, 0);

    buildWindow();
    doRefresh();
    syncMotionValues();
}

void GUIModule::uninit() {
    if (_window) {
        delete _window;
        _window = nullptr;
    }
    _client.disconnect();
}

void GUIModule::activate() {
    if (_window) _window->show();
}

void GUIModule::deactivate() {
    if (_window) _window->hide();
}

void GUIModule::update() {
    syncMotionValues();
    Fl::check();
}

void GUIModule::buildWindow() {
    Fl::scheme("gtk+");

    Fl::set_color(FL_BACKGROUND2_COLOR, 255, 255, 255);
    Fl::set_color(FL_FOREGROUND_COLOR, 255, 255, 255);

    const int w = 580, h = 650;
    _window = new Fl_Double_Window(w, h, "Log Configuration");
    _window->color(UITheme::BgColor);

    const int pad = 16;
    const int lh = 36;
    int y = pad;

    // Top Header
    Fl_Box *titleLabel = new Fl_Box(pad, y, 240, lh, "AVAILABLE SIGNALS");
    titleLabel->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    titleLabel->labelsize(14);
    titleLabel->labelfont(FL_HELVETICA_BOLD);
    titleLabel->labelcolor(UITheme::TextMuted);

    const int refreshBtnW = 130;
    _refreshBtn = new Fl_Button(w - pad - refreshBtnW, y, refreshBtnW, lh, "Refresh List");
    _refreshBtn->box(FL_ROUNDED_BOX);
    _refreshBtn->color(UITheme::BtnSecondary);
    _refreshBtn->labelcolor(FL_WHITE);
    _refreshBtn->labelsize(14);
    _refreshBtn->labelfont(FL_HELVETICA_BOLD);
    _refreshBtn->clear_visible_focus();
    _refreshBtn->callback(refreshCb, this);
    y += lh + 8;

    // Variables check browser
    const int browserH = 210;
    _varBrowser = new Fl_Check_Browser(pad, y, w - (2 * pad), browserH);
    _varBrowser->box(FL_FLAT_BOX);
    _varBrowser->color(UITheme::InputBg);
    
    // Checkbox and item styling:
    _varBrowser->selection_color(FL_WHITE); // Makes checkbox background/box white
    _varBrowser->textcolor(FL_WHITE);       // Text color
    _varBrowser->textsize(15);              // Noticeably larger item text (15pt)
    _varBrowser->textfont(FL_HELVETICA);
    y += browserH + pad + 6;

    // Output Configuration Section Header
    Fl_Box *outHeader = new Fl_Box(pad, y, 240, 24, "OUTPUT CONFIGURATION");
    outHeader->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    outHeader->labelsize(14);
    outHeader->labelfont(FL_HELVETICA_BOLD);
    outHeader->labelcolor(UITheme::TextMuted);
    y += 24 + 8;

    // Format selection row
    Fl_Box *formatLbl = new Fl_Box(pad, y, 70, lh, "Format");
    formatLbl->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    formatLbl->labelcolor(UITheme::TextMuted);
    formatLbl->labelsize(14);

    _formatChoice = new Fl_Choice(pad + 70, y, 160, lh, "");
    _formatChoice->box(FL_ROUNDED_BOX);
    _formatChoice->color(UITheme::SurfaceColor);
    _formatChoice->textcolor(FL_WHITE);
    _formatChoice->textsize(14);
    _formatChoice->labelsize(14);
    _formatChoice->clear_visible_focus();
    _formatChoice->add("ascii");
    _formatChoice->add("raw");
    _formatChoice->add("matlab");
    _formatChoice->value(0);
    _formatChoice->callback(formatChoiceCb, this);
    y += lh + 10;

    // File selection row
    Fl_Box *fileLbl = new Fl_Box(pad, y, 70, lh, "File");
    fileLbl->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    fileLbl->labelcolor(UITheme::TextMuted);
    fileLbl->labelsize(14);

    const int browseBtnW = 105;
    const int inputW = w - (2 * pad) - 70 - browseBtnW - 10;

    _fileInput = new Fl_Input(pad + 70, y, inputW, lh, "");
    _fileInput->box(FL_FLAT_BOX);
    _fileInput->color(UITheme::InputBg);
    _fileInput->textcolor(FL_WHITE);
    _fileInput->textsize(14);
    _fileInput->cursor_color(FL_WHITE);
    _fileInput->value(_outputFile.c_str());

    _browseBtn = new Fl_Button(pad + 70 + inputW + 10, y, browseBtnW, lh, "Browse...");
    _browseBtn->box(FL_ROUNDED_BOX);
    _browseBtn->color(UITheme::BtnSecondary);
    _browseBtn->labelcolor(FL_WHITE);
    _browseBtn->labelsize(14);
    _browseBtn->labelfont(FL_HELVETICA);
    _browseBtn->clear_visible_focus();
    _browseBtn->callback(browseCb, this);
    y += lh + pad + 6;

    // Apply Button
    const int applyBtnH = 40;
    _applyBtn = new Fl_Button(pad, y, w - (2 * pad), applyBtnH, "Apply Selection");
    _applyBtn->box(FL_ROUNDED_BOX);
    _applyBtn->color(UITheme::BtnPrimary);
    _applyBtn->labelcolor(FL_WHITE);
    _applyBtn->labelfont(FL_HELVETICA_BOLD);
    _applyBtn->labelsize(15);
    _applyBtn->clear_visible_focus();
    _applyBtn->callback(applyCb, this);
    y += applyBtnH + 10;

    // Status bar container
    const int statusH = 30;
    Fl_Box *statusContainer = new Fl_Box(pad, y, w - (2 * pad), statusH);
    statusContainer->box(FL_ROUNDED_BOX);
    statusContainer->color(UITheme::SurfaceColor);

    _statusBox = new Fl_Box(pad + 12, y, w - (2 * pad) - 24, statusH, "Ready");
    _statusBox->box(FL_NO_BOX);
    _statusBox->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    _statusBox->labelcolor(UITheme::TextMuted);
    _statusBox->labelsize(13);
    _statusBox->labelfont(FL_HELVETICA_ITALIC);

    y += statusH + 14;

    Fl_Box *motionHeader = new Fl_Box(pad, y, 260, 22, "CURRENT MOTION TARGETS");
    motionHeader->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    motionHeader->labelsize(13);
    motionHeader->labelfont(FL_HELVETICA_BOLD);
    motionHeader->labelcolor(UITheme::TextMuted);
    y += 22 + 8;

    // Left Column: Command Vel (x: pad to pad + 250)
    Fl_Box *vxLbl = new Fl_Box(pad, y, 110, lh, "Command Vel:");
    vxLbl->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    vxLbl->labelcolor(UITheme::TextMuted);
    vxLbl->labelsize(13);

    _vxOutput = new Fl_Output(pad + 110, y, 110, lh, "");
    _vxOutput->box(FL_ROUNDED_BOX);
    _vxOutput->color(UITheme::SurfaceColor);
    _vxOutput->textcolor(FL_WHITE);
    _vxOutput->textsize(13);
    _vxOutput->clear_visible_focus();

    // Right Column: Command Yaw Rate (x: pad + 245 to pad + 548)
    const int rightColX = pad + 245;
    Fl_Box *yawLbl = new Fl_Box(rightColX, y, 145, lh, "Command Yaw Rate:");
    yawLbl->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    yawLbl->labelcolor(UITheme::TextMuted);
    yawLbl->labelsize(13);

    _yawOutput = new Fl_Output(rightColX + 145, y, 110, lh, "");
    _yawOutput->box(FL_ROUNDED_BOX);
    _yawOutput->color(UITheme::SurfaceColor);
    _yawOutput->textcolor(FL_WHITE);
    _yawOutput->textsize(13);
    _yawOutput->clear_visible_focus();

    y += lh + pad;

    _window->end();
}

void GUIModule::doFormatChanged() {
    if (!_formatChoice || !_fileInput) return;

    // Determine target extension based on selection
    std::string newExt;
    int idx = _formatChoice->value();
    switch (idx) {
        case 0: newExt = ".csv"; break;  // or ".txt" / ".log"
        case 1: newExt = ".raw"; break;  // or ".bin"
        case 2: newExt = ".mat"; break;
        default: newExt = ".dat"; break;
    }

    std::string currentPath = _fileInput->value() ? _fileInput->value() : "";
    if (currentPath.empty()) {
        currentPath = "log" + newExt;
    } else {
        // Strip existing extension if present
        size_t dotPos = currentPath.find_last_of('.');
        size_t slashPos = currentPath.find_last_of("/\\");

        // Ensure the dot belongs to the filename, not a parent directory path
        if (dotPos != std::string::npos && (slashPos == std::string::npos || dotPos > slashPos)) {
            currentPath = currentPath.substr(0, dotPos);
        }
        currentPath += newExt;
    }

    _fileInput->value(currentPath.c_str());
    _outputFile = currentPath;
}

void GUIModule::syncMotionValues() {
    if (!_trotModule) {
        _trotModule = (MdlTrot *)_mgr->findModule(TROTMODULE_NAME, 0);
    }
    if (!_trotModule) return;

    char buf[32];

    if (_vxOutput) {
        std::snprintf(buf, sizeof(buf), "%.2f m/s", _trotModule->getForwardVel());
        _vxOutput->value(buf);
    }
    if (_yawOutput) {
        std::snprintf(buf, sizeof(buf), "%.2f rad/s", _trotModule->getYawRate());
        _yawOutput->value(buf);
    }
}


void GUIModule::populateVarBrowser() {
    if (!_varBrowser) return;
    _varBrowser->clear();
    for (const auto &v : _client.vars()) {
        _varBrowser->add(v.name.c_str(), 0);
    }
}

void GUIModule::doRefresh() {
    if (!_client.isConnected()) {
        if (_statusBox) _statusBox->copy_label("Not connected to LogServer");
        return;
    }
    if (_client.refreshVars()) {
        populateVarBrowser();
        if (_statusBox) {
            std::string msg = std::to_string(_client.vars().size()) + " variable(s) found";
            _statusBox->copy_label(msg.c_str());
        }
    } else {
        if (_statusBox) _statusBox->copy_label("Query failed");
    }
}

void GUIModule::doApply() {
    _selectedVars.clear();
    if (_varBrowser) {
        for (int i = 1; i <= _varBrowser->nitems(); i++) {
            if (_varBrowser->checked(i)) {
                _selectedVars.push_back(_varBrowser->text(i));
            }
        }
    }
    if (_formatChoice && _formatChoice->text()) {
        _logFormat = _formatChoice->text();
    }
    if (_fileInput && _fileInput->value()) {
        _outputFile = _fileInput->value();
    }

    // Ensure target folder exists before the backend tries to open the file
    if (!_outputFile.empty()) {
        std::error_code ec;
        std::filesystem::path filePath(_outputFile);
        std::filesystem::path parentDir = filePath.parent_path();

        if (!parentDir.empty() && !std::filesystem::exists(parentDir, ec)) {
            std::filesystem::create_directories(parentDir, ec);
            if (ec) {
                if (_statusBox) {
                    _statusBox->copy_label("Failed to create directory!");
                }
                return;
            }
        }
    }

    _ready = true;

    if (_statusBox) {
        std::string msg = std::to_string(_selectedVars.size()) + " var(s) applied";
        _statusBox->copy_label(msg.c_str());
    }
}

void GUIModule::doBrowse() {
    Fl_Native_File_Chooser fnfc;
    fnfc.title("Choose Log Output File");
    fnfc.type(Fl_Native_File_Chooser::BROWSE_SAVE_FILE);
    fnfc.options(Fl_Native_File_Chooser::SAVEAS_CONFIRM);
    
    int selectedIdx = _formatChoice ? _formatChoice->value() : 0;
    
    switch (selectedIdx) {
        case 0: // ascii
            fnfc.filter("Text/Log Files (*.txt;*.log;*.csv)\t*.txt;*.log;*.csv");
            break;
            case 1: // raw
            fnfc.filter("Binary Data Files (*.raw;*.bin)\t*.raw;*.bin");
            break;
        case 2: // matlab
        fnfc.filter("MATLAB MAT-Files (*.mat)\t*.mat");
        break;
    }
    
    if (_fileInput && _fileInput->value() && _fileInput->value()[0] != '\0') {
        fnfc.preset_file(_fileInput->value());
    }
    
    if (fnfc.show() == 0) {
        if (_fileInput) {
            _fileInput->value(fnfc.filename());
        }
    }
}
void GUIModule::formatChoiceCb(Fl_Widget *, void *v) {
    static_cast<GUIModule *>(v)->doFormatChanged();
}

void GUIModule::browseCb(Fl_Widget *, void *v) {
    static_cast<GUIModule *>(v)->doBrowse();
}

void GUIModule::refreshCb(Fl_Widget *, void *v) {
    static_cast<GUIModule *>(v)->doRefresh();
}

void GUIModule::applyCb(Fl_Widget *, void *v) {
    static_cast<GUIModule *>(v)->doApply();
}