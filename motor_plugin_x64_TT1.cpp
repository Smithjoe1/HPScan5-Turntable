#define WIN32_LEAN_AND_MEAN
#include <atomic>
#include <cmath>
#include <windows.h>
#include <string>
#include <fstream>
#include <ctime>
#include <cstdio>

// ============================================================
//  DEBUG LEVEL
//  0 = errors + connect/disconnect/move only (minimal)
//  1 = + getStatus changes, getParameter calls
//  2 = + every getStatus call, every serial TX/RX
//  3 = + every motor_getPosition call (very verbose)
// ============================================================
#define DEBUG_LEVEL 0

// ============================================================
//  LOGGING
// ============================================================
static std::ofstream    g_log;
static CRITICAL_SECTION g_logCS;
static bool             g_logCSInit = false;

static void log_open() {
    if (!g_logCSInit) { InitializeCriticalSection(&g_logCS); g_logCSInit = true; }
    EnterCriticalSection(&g_logCS);
    g_log.open("C:\\Program Files\\HP 3D Scan 5\\logs\\bypass_plugin.log", std::ios::app);
    time_t t = time(nullptr);
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&t));
    g_log << "\n============================\n" << buf
          << "  [DEBUG_LEVEL=" << DEBUG_LEVEL << "]\n";
    g_log.flush();
    LeaveCriticalSection(&g_logCS);
}

static void logL(int level, const std::string& s) {
    if (level > DEBUG_LEVEL) return;
    if (!g_log.is_open()) return;
    char buf[64];
    snprintf(buf, sizeof(buf), "[T%05lu @%lu] ",
             (unsigned long)GetCurrentThreadId(),
             (unsigned long)GetTickCount());
    EnterCriticalSection(&g_logCS);
    g_log << buf << s << "\n";
    g_log.flush();
    LeaveCriticalSection(&g_logCS);
}
static void log(const std::string& s) { logL(0, s); }

// ============================================================
//  SERIAL HELPERS
// ============================================================
static bool serial_write(HANDLE h, const std::string& s) {
    std::string display = s;
    while (!display.empty() && (display.back()=='\n'||display.back()=='\r'))
        display.pop_back();
    logL(2, "  TX: [" + display + "]");
    DWORD written = 0;
    bool ok = WriteFile(h, s.c_str(), (DWORD)s.size(), &written, nullptr)
              && written == (DWORD)s.size();
    if (!ok) log("  TX FAILED written=" + std::to_string(written)
                 + " err=" + std::to_string(GetLastError()));
    return ok;
}

static std::string serial_readline(HANDLE h, DWORD timeout_ms = 2000) {
    std::string result;
    char c = 0; DWORD rd = 0;
    DWORD start = GetTickCount();
    while (GetTickCount() - start < timeout_ms) {
        if (ReadFile(h, &c, 1, &rd, nullptr) && rd == 1) {
            if (c == '\n') break;
            if (c != '\r') result += c;
        }
    }
    logL(2, "  RX: [" + result + "]"
         + (result.empty() ? " (timeout/empty)" : ""));
    return result;
}

static HANDLE open_com(const std::string& port) {
    std::string path = "\\\\.\\" + port;
    logL(1, "  Opening " + path);
    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                           0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        log("  open_com FAILED port=" + port
            + " err=" + std::to_string(GetLastError()));
        return h;
    }
    DCB dcb = {}; dcb.DCBlength = sizeof(DCB);
    GetCommState(h, &dcb);
    dcb.BaudRate = 115200; dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY; dcb.StopBits = ONESTOPBIT;
    SetCommState(h, &dcb);
    COMMTIMEOUTS to = {};
    to.ReadIntervalTimeout       = 10;
    to.ReadTotalTimeoutConstant  = 500;
    to.WriteTotalTimeoutConstant = 1000;
    SetCommTimeouts(h, &to);
    // Assert DTR — TinyUSB CDC on the Pico will not flush its TX buffer
    // to the host until DTR is set. Serial monitor sets it automatically;
    // we must set it explicitly here or the Pico's responses never arrive.
    EscapeCommFunction(h, SETDTR);
    logL(1, "  Opened OK (DTR asserted)");
    return h;
}

static std::string find_pico_port(double& stepsPerDeg) {
    log("find_pico_port: scanning...");
    HKEY hKey;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM",
                      0, KEY_READ, &hKey) != ERROR_SUCCESS) return "";
    char valName[256], valData[256];
    DWORD idx = 0, nameLen, dataLen, type;
    std::string found;
    while (true) {
        nameLen = sizeof(valName); dataLen = sizeof(valData);
        if (RegEnumValueA(hKey, idx++, valName, &nameLen, nullptr, &type,
                          (LPBYTE)valData, &dataLen) != ERROR_SUCCESS) break;
        std::string portName(valData, dataLen > 0 ? dataLen-1 : 0);
        HANDLE h = open_com(portName);
        if (h == INVALID_HANDLE_VALUE) continue;
        serial_write(h, "\n"); Sleep(200);
        PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR); Sleep(100);
        serial_write(h, "steps_per_degree\n");
        std::string resp = serial_readline(h, 1500);
        CloseHandle(h);
        if (!resp.empty() && resp.find('.') != std::string::npos) {
            try { stepsPerDeg = std::stod(resp); } catch (...) {}
            found = portName;
            log("find_pico_port: found on " + portName);
            break;
        }
    }
    RegCloseKey(hKey);
    return found;
}

// ============================================================
//  INSTANCE
//
//  KEY INSIGHT from log analysis:
//  The inst pointer passed to motor_move (and motor_getStatus after
//  the move) is DIFFERENT from the one returned by motor_connect.
//  HP is passing its own internal wrapper object pointer, not our
//  MotorInstance*. This caused the write-protected-memory crash —
//  the move thread was writing into HP's memory via a garbage hCom.
//
//  Fix: all exported functions resolve the actual MotorInstance via
//  resolve_inst(), which validates the pointer against g_current_inst.
//  If the pointer doesn't match, we use g_current_inst directly.
//  g_current_inst is set at connect time and is always correct.
// ============================================================
struct MotorInstance;
static MotorInstance* g_current_inst = nullptr;
static std::atomic<bool> g_move_in_progress{false};

typedef unsigned long long (*pfn_destroy)      (MotorInstance*, int);
typedef unsigned long long (*pfn_connect_int)  (MotorInstance*, void*, void*);
typedef unsigned long long (*pfn_no_param)     ();
typedef unsigned long long (*pfn_move)         (MotorInstance*, double);
typedef unsigned long long (*pfn_getPosition)  (MotorInstance*, void*);
typedef unsigned long long (*pfn_getParameter) (MotorInstance*, void*, void*);
typedef unsigned long long (*pfn_setParameter) (MotorInstance*, void*, void*);

struct VTable {
    pfn_destroy      destroy;
    pfn_connect_int  connect_internal;
    pfn_no_param     disconnect_internal;
    pfn_move         move;
    pfn_no_param     stop;
    pfn_getPosition  getPosition;
    pfn_no_param     getStatus;
    pfn_getParameter getParameter;
    pfn_setParameter setParameter;
};

struct MotorInstance {
    VTable*           vtable;
    HANDLE            hCom            = INVALID_HANDLE_VALUE;
    bool              connected       = false;
    std::atomic<bool> moving{false};
    std::atomic<bool> cancelled{false};
    HANDLE            hMoveThread     = nullptr;
    double            position        = 0.0;
    double            stepsPerDeg     = 57.4359;
    double            speed           = 50.0;
    int               numScans        = 9;
    std::string       port;
    int               statusCallCount = 0;
    bool              initDone        = false;
    DWORD             magic           = 0xDEADBEEF;  // used to validate pointer
};

// Validate an inst pointer: if it looks like our real instance, use it.
// Otherwise fall back to g_current_inst. Logs whenever a mismatch occurs.
static MotorInstance* resolve_inst(void* raw, const char* caller) {
    MotorInstance* inst = static_cast<MotorInstance*>(raw);
    if (inst && inst == g_current_inst && inst->magic == 0xDEADBEEF) {
        return inst;  // pointer is valid and matches our instance
    }
    // Pointer is wrong — HP passed its own wrapper or a stale pointer
    if (inst != g_current_inst) {
        log(std::string(caller) + ": inst mismatch!"
            + " got="    + std::to_string((unsigned long long)inst)
            + " expect=" + std::to_string((unsigned long long)g_current_inst)
            + " → using g_current_inst");
    }
    return g_current_inst;
}

// ============================================================
//  BACKGROUND MOVE THREAD
// ============================================================
struct MoveParams {
    MotorInstance* inst;
    double         angle;
};

static DWORD WINAPI move_thread(LPVOID param) {
    MoveParams* p    = static_cast<MoveParams*>(param);
    MotorInstance* inst = p->inst;
    double angle     = p->angle;
    delete p;

    log("move_thread START angle=" + std::to_string(angle)
        + " hCom=" + std::to_string((unsigned long long)inst->hCom));

    if (inst->hCom == INVALID_HANDLE_VALUE) {
        log("move_thread ERROR: hCom is INVALID_HANDLE_VALUE — aborting");
        inst->moving = false;
        g_move_in_progress = false;
        return 1;
    }

    if (!inst->cancelled.load()) {
        if (fabs(angle) < 0.5) {
            log("move_thread: homing");
            serial_write(inst->hCom, "P\n");
            inst->position = 0.0;
            DWORD start = GetTickCount();
            while (!inst->cancelled.load() && GetTickCount() - start < 10000) {
                std::string resp = serial_readline(inst->hCom, 500);
                if (resp.find("HOME") != std::string::npos) {
                    log("move_thread: HOME confirmed"); break;
                }
            }
        } else {
            char buf[64];
            if (angle >= 0) snprintf(buf, sizeof(buf), "m+%.4f\n", angle);
            else            snprintf(buf, sizeof(buf), "m%.4f\n",  angle);
            serial_write(inst->hCom, buf);
            inst->position += angle;

            DWORD timeout = (DWORD)(fabs(angle) / 360.0 * 25000.0) + 3000;
            if (timeout > 35000) timeout = 35000;
            log("move_thread: waiting for DONE (timeout="
                + std::to_string(timeout) + "ms)");

            DWORD start = GetTickCount();
            bool gotDone = false;
            while (!inst->cancelled.load() && GetTickCount() - start < timeout) {
                std::string resp = serial_readline(inst->hCom, 500);
                if (resp.find("DONE") != std::string::npos) {
                    log("move_thread: DONE received after "
                        + std::to_string(GetTickCount()-start) + "ms");
                    gotDone = true;
                    break;
                }
            }
            if (!gotDone)
                log("move_thread: DONE never received (timeout or cancelled)");
        }
    } else {
        log("move_thread: cancelled before start");
    }

    inst->moving = false;
    g_move_in_progress = false;
    log("move_thread END — status now 0 (ready)");
    return 0;
}

// ============================================================
//  VTABLE IMPLEMENTATIONS
// ============================================================
static unsigned long long impl_destroy(MotorInstance* self, int mode) {
    log("impl_destroy mode=" + std::to_string(mode));
    self = static_cast<MotorInstance*>(resolve_inst(self, "impl_destroy"));
    if (!self) return 0;

    self->cancelled = true;
    self->magic     = 0;  // invalidate so resolve_inst won't match after free

    if (self->hMoveThread) {
        log("impl_destroy: waiting for move thread...");
        DWORD r = WaitForSingleObject(self->hMoveThread, 36000);
        log("impl_destroy: wait=" + std::string(r==WAIT_OBJECT_0?"OK":r==WAIT_TIMEOUT?"TIMEOUT":"ERR"));
        CloseHandle(self->hMoveThread);
        self->hMoveThread = nullptr;
    }
    if (self->hCom != INVALID_HANDLE_VALUE) {
        serial_write(self->hCom, "C\n"); Sleep(100);
        CloseHandle(self->hCom);
        self->hCom = INVALID_HANDLE_VALUE;
    }
    self->connected = false;
    if (g_current_inst == self) g_current_inst = nullptr;
    if (mode == 1) { log("impl_destroy: delete inst"); delete self; }
    return 0;
}

static unsigned long long impl_connect_internal(MotorInstance*, void*, void*) { return 0; }

static unsigned long long impl_disconnect_internal() {
    log("impl_disconnect_internal");
    MotorInstance* self = g_current_inst;
    if (!self) return 0;
    self->cancelled = true;
    if (self->hMoveThread) {
        WaitForSingleObject(self->hMoveThread, 36000);
        CloseHandle(self->hMoveThread);
        self->hMoveThread = nullptr;
    }
    if (self->hCom != INVALID_HANDLE_VALUE) {
        serial_write(self->hCom, "C\n"); Sleep(100);
        CloseHandle(self->hCom);
        self->hCom = INVALID_HANDLE_VALUE;
    }
    self->connected = false;
    return 0;
}

// The vtable move slot — HP may call move via vtable dispatch.
// Forward to the same logic used by the motor_move export.
// Defined after do_move below.
static unsigned long long do_move(MotorInstance* inst, double angle);
static unsigned long long impl_move(MotorInstance* self, double angle) {
    log("impl_move (vtable) angle=" + std::to_string(angle));
    self = resolve_inst(self, "impl_move");
    return do_move(self, angle);
}

static unsigned long long impl_stop() {
    log("impl_stop");
    MotorInstance* self = g_current_inst;
    if (!self) return 0ULL;
    self->cancelled = true;
    if (self->hCom != INVALID_HANDLE_VALUE)
        serial_write(self->hCom, "C\n");
    self->moving = false;
    g_move_in_progress = false;
    return 0ULL;
}

static unsigned long long impl_getPosition(MotorInstance* self, void* output) {
    self = resolve_inst(self, "impl_getPosition");
    if (!self) return (unsigned long long)(int)-10;
    logL(3, "impl_getPosition=" + std::to_string(self->position));
    if (output) *(double*)output = self->position;
    return 0ULL;
}

static int   g_last_logged_status  = -999;
static DWORD g_last_status_log_tick = 0;

static unsigned long long impl_getStatus() {
    if (!g_current_inst || !g_current_inst->connected)
        return (unsigned long long)(int)-10;

    int status;
    if (g_current_inst->moving.load()) {
        status = 1;
    } else if (!g_current_inst->initDone) {
        g_current_inst->statusCallCount++;
        status = (g_current_inst->statusCallCount >= 20) ? 0 : -4;
        if (g_current_inst->statusCallCount == 20) g_current_inst->initDone = true;
    } else {
        status = 0;
    }

    bool changed = (status != g_last_logged_status);
    if (changed) {
        logL(1, "impl_getStatus=" + std::to_string(status)
             + " (was " + std::to_string(g_last_logged_status) + ")");
        g_last_logged_status   = status;
        g_last_status_log_tick = GetTickCount();
    } else if (DEBUG_LEVEL >= 2) {
        DWORD now = GetTickCount();
        if (now - g_last_status_log_tick >= 500) {
            logL(2, "impl_getStatus=" + std::to_string(status) + " (unchanged)");
            g_last_status_log_tick = now;
        }
    }
    return (unsigned long long)(int)status;
}

static unsigned long long impl_getParameter(MotorInstance*, void*, void*) {
    return (unsigned long long)(int)-1;
}
static unsigned long long impl_setParameter(MotorInstance*, void*, void*) { return 0ULL; }

static VTable g_vtable = {
    impl_destroy, impl_connect_internal, impl_disconnect_internal,
    impl_move, impl_stop, impl_getPosition, impl_getStatus,
    impl_getParameter, impl_setParameter
};

// ============================================================
//  CORE MOVE LOGIC  (shared by vtable and export)
// ============================================================
static unsigned long long do_move(MotorInstance* inst, double angle) {
    if (!inst) {
        log("do_move ERROR: null inst");
        return (unsigned long long)(int)-10;
    }
    if (!inst->connected || inst->hCom == INVALID_HANDLE_VALUE) {
        log("do_move ERROR: not connected or hCom invalid"
            " connected=" + std::to_string(inst->connected)
            + " hCom=" + std::to_string((unsigned long long)inst->hCom));
        return (unsigned long long)(int)-10;
    }

    bool expected = false;
    if (!g_move_in_progress.compare_exchange_strong(expected, true)) {
        log("do_move SKIPPED (already in progress)");
        return 0ULL;
    }

    if (angle >  360.0) angle =  360.0;
    if (angle < -360.0) angle = -360.0;
    log("do_move: angle=" + std::to_string(angle)
        + " hCom=" + std::to_string((unsigned long long)inst->hCom));

    inst->cancelled = false;
    inst->moving    = true;
    inst->initDone  = true;
    g_last_logged_status = -999;

    MoveParams* p  = new MoveParams{inst, angle};
    HANDLE hThread = CreateThread(nullptr, 0, move_thread, p, 0, nullptr);
    if (!hThread) {
        log("do_move ERROR: CreateThread failed err="
            + std::to_string(GetLastError()));
        delete p;
        inst->moving       = false;
        g_move_in_progress = false;
        return (unsigned long long)(int)-10;
    }
    inst->hMoveThread = hThread;
    log("do_move: thread launched handle="
        + std::to_string((unsigned long long)hThread));
    return 0ULL;
}

// ============================================================
//  EXPORTED FUNCTIONS
// ============================================================
extern "C" {

__declspec(dllexport)
unsigned long long __cdecl motor_connect(void** ppPlugin, const char* portName,
                                          void* /*unknown*/, const char* params) {
    log_open();
    log("motor_connect");
    log("  portName=[" + std::string(portName ? portName : "(null)") + "]");
    log("  params=["   + std::string(params   ? params   : "(null)") + "]");
    if (!ppPlugin) { log("  ERROR: ppPlugin null"); return (unsigned long long)(int)-10; }

    MotorInstance* inst   = new MotorInstance();
    inst->vtable          = &g_vtable;
    inst->statusCallCount = 0;
    inst->initDone        = false;
    inst->cancelled       = false;
    inst->hMoveThread     = nullptr;
    inst->magic           = 0xDEADBEEF;
    g_last_logged_status  = -999;
    g_move_in_progress    = false;

    std::string port;
    if (portName && portName[0] != '\0') {
        port = portName;
        if (port.size() >= 5 && port.substr(0,5) == "port:") port = port.substr(5);
        while (!port.empty() && isspace((unsigned char)port.back())) port.pop_back();
        log("  port from portName: [" + port + "]");
    }
    if (port.empty()) port = find_pico_port(inst->stepsPerDeg);
    if (port.empty()) {
        log("  ERROR: no port"); delete inst;
        return (unsigned long long)(int)-10;
    }
    inst->port = port;

    HANDLE h = INVALID_HANDLE_VALUE;
    for (int i = 0; i < 10; i++) {
        h = open_com(port);
        if (h != INVALID_HANDLE_VALUE) break;
        log("  retry " + std::to_string(i+1) + "/10"); Sleep(300);
    }
    if (h == INVALID_HANDLE_VALUE) {
        log("  ERROR: could not open COM port");
        delete inst; return (unsigned long long)(int)-10;
    }

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    // Give the Pico time to recognise DTR assertion and prepare to respond
    Sleep(800);
    serial_write(h, "steps_per_degree\n");
    std::string r1 = serial_readline(h, 3000);
    if (r1.empty()) {
        log("  ERROR: no response to steps_per_degree");
        CloseHandle(h); delete inst; return (unsigned long long)(int)-10;
    }
    try { inst->stepsPerDeg = std::stod(r1); } catch (...) {
        log("  WARNING: stod failed on [" + r1 + "]");
    }
    log("  stepsPerDeg=" + r1);
    serial_write(h, "version\n");
    std::string r2 = serial_readline(h, 2000);
    log("  version=[" + r2 + "]");

    inst->hCom      = h;
    inst->connected = true;
    inst->position  = 0.0;
    g_current_inst  = inst;
    *ppPlugin       = inst;
    log("motor_connect SUCCESS"
        " inst=" + std::to_string((unsigned long long)inst)
        + " hCom=" + std::to_string((unsigned long long)h));
    return 0ULL;
}

__declspec(dllexport)
void __cdecl motor_disconnect(MotorInstance* inst) {
    log("motor_disconnect inst=" + std::to_string((unsigned long long)inst));
    inst = resolve_inst(inst, "motor_disconnect");
    if (!inst) { log("  WARNING: no inst to disconnect"); return; }
    g_current_inst = inst;
    inst->vtable->disconnect_internal();
    inst->vtable->destroy(inst, 1);
    log("motor_disconnect DONE");
}

__declspec(dllexport)
unsigned long long __cdecl motor_move(MotorInstance* inst, double angle) {
    log("motor_move ENTER"
        " inst=" + std::to_string((unsigned long long)inst)
        + " angle=" + std::to_string(angle));
    inst = resolve_inst(inst, "motor_move");
    return do_move(inst, angle);
}

__declspec(dllexport)
unsigned long long __cdecl motor_getPosition(long long output, MotorInstance* inst) {
    inst = resolve_inst(inst, "motor_getPosition");
    logL(3, "motor_getPosition inst=" + std::to_string((unsigned long long)inst));
    if (!inst) return (unsigned long long)(int)-10;
    g_current_inst = inst;
    if (output) *(double*)output = inst->position;
    return 0ULL;
}

__declspec(dllexport)
unsigned long long __cdecl motor_getStatus(MotorInstance* inst) {
    // Note: do NOT call resolve_inst here and log every mismatch —
    // getStatus is called every 5ms and any mismatch would flood the log.
    // Instead just always use g_current_inst via impl_getStatus.
    logL(2, "motor_getStatus inst=" + std::to_string((unsigned long long)inst));
    if (!inst && !g_current_inst) return (unsigned long long)(int)-10;
    return impl_getStatus();
}

__declspec(dllexport)
unsigned long long __cdecl motor_stop(MotorInstance* inst) {
    log("motor_stop inst=" + std::to_string((unsigned long long)inst));
    return impl_stop();
}

__declspec(dllexport)
unsigned long long __cdecl motor_getParameter(long long* output, const char* name,
                                               MotorInstance* inst) {
    logL(1, "motor_getParameter name=[" + std::string(name ? name : "null") + "]"
         + " inst=" + std::to_string((unsigned long long)inst));
    inst = resolve_inst(inst, "motor_getParameter");
    if (!inst || !name) return (unsigned long long)(int)-1;
    g_current_inst = inst;
    std::string n(name);
    if (n == "tilt" || n == "tiltAvailable") { if (output) *output = 0LL; return 0ULL; }
    if (n == "speed")          { if (output) *output = (long long)inst->speed;    return 0ULL; }
    if (n == "numScans")       { if (output) *output = (long long)inst->numScans; return 0ULL; }
    if (n == "stepsPerDegree") {
        if (output) memcpy(output, &inst->stepsPerDeg, sizeof(double));
        return 0ULL;
    }
    logL(1, "  unknown param [" + n + "]");
    return (unsigned long long)(int)-1;
}

__declspec(dllexport)
unsigned long long __cdecl motor_setParameter(void* p1, void* p2, void* p3) {
    logL(1, "motor_setParameter p1=" + std::to_string((unsigned long long)p1)
         + " p2=" + std::to_string((unsigned long long)p2)
         + " p3=" + std::to_string((unsigned long long)p3));
    if (p3) {
        MotorInstance* inst = resolve_inst(p3, "motor_setParameter");
        if (inst) g_current_inst = inst;
    }
    return 0ULL;
}

} // extern "C"

BOOL WINAPI DllMain(HINSTANCE, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_DETACH && g_log.is_open()) g_log.close();
    return TRUE;
}
