#include "serial_transport.hpp"

#if defined(_WIN32)
#include <windows.h>
#include <initguid.h>  // before devpkey.h: defines the DEVPKEY_* constants here
#include <devguid.h>
#include <devpkey.h>
#include <setupapi.h>
#else
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#endif
#endif

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <set>
#include <system_error>

namespace gcs::link {

namespace fs = std::filesystem;

const std::vector<int>& serialBauds() {
    static const std::vector<int> b = {57600, 115200, 230400, 460800, 921600};
    return b;
}

namespace {

#if !defined(_WIN32)
std::string sysError(const std::string& what) {
    return what + ": " + std::strerror(errno);
}
#endif

#if !defined(__APPLE__) && !defined(_WIN32)
bool speedConstant(int baud, speed_t& out) {
    switch (baud) {
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
#ifdef B460800
    case 460800: out = B460800; return true;
#endif
#ifdef B921600
    case 921600: out = B921600; return true;
#endif
#ifdef B1500000
    case 1500000: out = B1500000; return true;
#endif
    default: return false;
    }
}
#endif

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool looksLikeAutopilot(const std::string& name) {
    const std::string n = lower(name);
    for (const char* k : {"pixhawk", "ardupilot", "px4", "cube", "fmu", "holybro", "matek", "speedybee", "kakute",
                          "mro", "cuav", "durandal", "pixracer"}) {
        if (n.find(k) != std::string::npos) {
            return true;
        }
    }
    return false;
}

#if defined(__linux__)
// "usb-ArduPilot_Pixhawk6C_340036000F51333131383532-if00" -> "ArduPilot Pixhawk6C"
std::string labelFromById(std::string id) {
    if (id.rfind("usb-", 0) == 0) {
        id = id.substr(4);
    }
    const auto ifpos = id.rfind("-if");
    if (ifpos != std::string::npos) {
        id = id.substr(0, ifpos);
    }
    const auto last = id.rfind('_');
    if (last != std::string::npos && id.size() - last > 8) {
        id = id.substr(0, last);  // a long trailing serial number
    }
    std::replace(id.begin(), id.end(), '_', ' ');
    return id;
}
#endif

}  // namespace

// ---- POSIX (Linux, macOS) ---------------------------------------------------------------------
#if !defined(_WIN32)

SerialTransport::SerialTransport(std::string port, int baud) : port_(std::move(port)), baud_(baud) {}

SerialTransport::~SerialTransport() {
    close();
}

bool SerialTransport::isOpen() const {
    return fd_ >= 0;
}

bool SerialTransport::open(std::string& error) {
    close();
    if (port_.empty()) {
        error = "no serial port chosen";
        return false;
    }
    const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        if (errno == ENOENT) {
            error = port_ + " not found (unplugged?)";
        } else if (errno == EACCES) {
            error = "no permission to open " + port_ +
#if defined(__linux__)
                    " (add yourself to the dialout group, then log in again)";
#else
                    "";
#endif
        } else if (errno == EBUSY) {
            error = port_ + " is in use by another program";
        } else {
            error = sysError("open " + port_);
        }
        return false;
    }
    if (::ioctl(fd, TIOCEXCL) < 0 && errno != ENOTTY) {
        error = port_ + " is in use by another program";
        ::close(fd);
        return false;
    }
    termios tio{};
    if (::tcgetattr(fd, &tio) < 0) {
        error = sysError("tcgetattr " + port_);
        ::close(fd);
        return false;
    }
    ::cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | CRTSCTS);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
#if defined(__APPLE__)
    ::cfsetspeed(&tio, B9600);  // placeholder; the real rate is set below (any value works)
#else
    speed_t sp{};
    if (!speedConstant(baud_, sp)) {
        error = "baud " + std::to_string(baud_) + " not supported";
        ::close(fd);
        return false;
    }
    ::cfsetispeed(&tio, sp);
    ::cfsetospeed(&tio, sp);
#endif
    if (::tcsetattr(fd, TCSANOW, &tio) < 0) {
        error = sysError("tcsetattr " + port_);
        ::close(fd);
        return false;
    }
#if defined(__APPLE__)
    speed_t speed = static_cast<speed_t>(baud_);
    if (::ioctl(fd, IOSSIOSPEED, &speed) < 0) {
        error = sysError("baud " + std::to_string(baud_));
        ::close(fd);
        return false;
    }
#endif
    ::tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    return true;
}

void SerialTransport::close() {
    if (fd_ >= 0) {
        ::close(static_cast<int>(fd_));
        fd_ = -1;
    }
}

int SerialTransport::waitReadable(int timeout_ms, std::string& error) {
    if (fd_ < 0) {
        error = "port closed";
        return -1;
    }
    pollfd p{};
    p.fd = static_cast<int>(fd_);
    p.events = POLLIN;
    const int ready = ::poll(&p, 1, timeout_ms);
    if (ready < 0) {
        if (errno == EINTR) {
            return 0;
        }
        error = sysError("poll " + port_);
        return -1;
    }
    if (ready > 0 && (p.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        // unplugged (a hang-up can also flag POLLIN, then read() returns 0 forever)
        error = port_ + " disconnected";
        return -1;
    }
    return ready > 0 ? 1 : 0;
}

long SerialTransport::read(uint8_t* buf, size_t max, std::string& error) {
    if (fd_ < 0) {
        error = "port closed";
        return -1;
    }
    const auto n = ::read(static_cast<int>(fd_), buf, max);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return 0;
        }
        error = errno == EIO || errno == ENXIO || errno == ENODEV ? port_ + " disconnected" : sysError("read " + port_);
        return -1;
    }
    return static_cast<long>(n);
}

void SerialTransport::write(const uint8_t* data, size_t len) {
    if (fd_ < 0) {
        return;
    }
    // a whole MAVLink frame or nothing useful: wait briefly when the driver's buffer is full
    size_t done = 0;
    for (int tries = 0; done < len && tries < 5; ++tries) {
        const auto n = ::write(static_cast<int>(fd_), data + done, len - done);
        if (n > 0) {
            done += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            return;  // the read side notices a vanished port
        }
        pollfd p{};
        p.fd = static_cast<int>(fd_);
        p.events = POLLOUT;
        ::poll(&p, 1, 10);
    }
}

std::string SerialTransport::describe() const {
    return port_ + " · " + std::to_string(baud_);
}

namespace {
std::vector<SerialPortInfo> platformPorts() {
    std::vector<SerialPortInfo> out;
    std::error_code ec;
#if defined(__linux__)
    std::set<std::string> seen;
    // udev's stable names carry the USB product: "usb-ArduPilot_Pixhawk6C_...-if00"
    for (const auto& e : fs::directory_iterator("/dev/serial/by-id", ec)) {
        const fs::path target = fs::canonical(e.path(), ec);
        if (ec) {
            continue;
        }
        SerialPortInfo p;
        p.path = target.string();
        p.label = labelFromById(e.path().filename().string()) + " (" + target.filename().string() + ")";
        p.likely_autopilot = looksLikeAutopilot(p.label);
        if (seen.insert(p.path).second) {
            out.push_back(std::move(p));
        }
    }
    ec.clear();
    for (const auto& e : fs::directory_iterator("/dev", ec)) {
        const std::string name = e.path().filename().string();
        if (name.rfind("ttyACM", 0) == 0 || name.rfind("ttyUSB", 0) == 0) {
            const std::string path = e.path().string();
            if (seen.insert(path).second) {
                out.push_back({path, name, false});
            }
        }
    }
#elif defined(__APPLE__)
    // callout devices (cu.*): tty.* would wait for a carrier signal on open
    for (const auto& e : fs::directory_iterator("/dev", ec)) {
        const std::string name = e.path().filename().string();
        if (name.rfind("cu.", 0) != 0 || name.find("Bluetooth") != std::string::npos ||
            name.find("debug-console") != std::string::npos || name.find("wlan") != std::string::npos) {
            continue;
        }
        SerialPortInfo p;
        p.path = e.path().string();
        p.label = name.substr(3);
        p.likely_autopilot = name.find("usbmodem") != std::string::npos || looksLikeAutopilot(name);
        out.push_back(std::move(p));
    }
#endif
    return out;
}
}  // namespace

#else
// ---- Windows ------------------------------------------------------------------------------------

namespace {
HANDLE handle(intptr_t h) { return reinterpret_cast<HANDLE>(h); }
constexpr intptr_t kNoPort = -1;  // INVALID_HANDLE_VALUE

std::string winError(DWORD err) {
    char buf[256] = {};
    const DWORD n = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, err, 0, buf,
                                   sizeof(buf), nullptr);
    std::string s(buf, n);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '.')) {
        s.pop_back();
    }
    return s.empty() ? "error " + std::to_string(err) : s;
}
}  // namespace

SerialTransport::SerialTransport(std::string port, int baud) : port_(std::move(port)), baud_(baud) {}

SerialTransport::~SerialTransport() {
    close();
}

bool SerialTransport::isOpen() const {
    return fd_ != kNoPort;
}

bool SerialTransport::open(std::string& error) {
    close();
    if (port_.empty()) {
        error = "no serial port chosen";
        return false;
    }
    // COM10 and above only open through the device namespace
    const std::string dev = port_.rfind("\\\\.\\", 0) == 0 ? port_ : "\\\\.\\" + port_;
    HANDLE h = ::CreateFileA(dev.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        const DWORD err = ::GetLastError();
        if (err == ERROR_FILE_NOT_FOUND || err == ERROR_PATH_NOT_FOUND) {
            error = port_ + " not found (unplugged?)";
        } else if (err == ERROR_ACCESS_DENIED || err == ERROR_SHARING_VIOLATION) {
            error = port_ + " is in use by another program";
        } else {
            error = "open " + port_ + ": " + winError(err);
        }
        return false;
    }
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!::GetCommState(h, &dcb)) {
        error = "GetCommState " + port_ + ": " + winError(::GetLastError());
        ::CloseHandle(h);
        return false;
    }
    dcb.BaudRate = static_cast<DWORD>(baud_);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fNull = FALSE;
    dcb.fAbortOnError = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;  // USB CDC devices expect DTR before they talk
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!::SetCommState(h, &dcb)) {
        error = "baud " + std::to_string(baud_) + " on " + port_ + ": " + winError(::GetLastError());
        ::CloseHandle(h);
        return false;
    }
    // reads return at once with whatever has arrived; writes give up after 50 ms
    COMMTIMEOUTS t{};
    t.ReadIntervalTimeout = MAXDWORD;
    t.ReadTotalTimeoutMultiplier = 0;
    t.ReadTotalTimeoutConstant = 0;
    t.WriteTotalTimeoutMultiplier = 0;
    t.WriteTotalTimeoutConstant = 50;
    ::SetCommTimeouts(h, &t);
    ::PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    fd_ = reinterpret_cast<intptr_t>(h);
    return true;
}

void SerialTransport::close() {
    if (fd_ != kNoPort) {
        ::CloseHandle(handle(fd_));
        fd_ = kNoPort;
    }
}

int SerialTransport::waitReadable(int timeout_ms, std::string& error) {
    if (fd_ == kNoPort) {
        error = "port closed";
        return -1;
    }
    // COM ports have no poll(): look at the input queue every millisecond
    const DWORD start = ::GetTickCount();
    for (;;) {
        DWORD errors = 0;
        COMSTAT st{};
        if (!::ClearCommError(handle(fd_), &errors, &st)) {
            error = port_ + " disconnected";
            return -1;
        }
        if (st.cbInQue > 0) {
            return 1;
        }
        if (static_cast<int>(::GetTickCount() - start) >= timeout_ms) {
            return 0;
        }
        ::Sleep(1);
    }
}

long SerialTransport::read(uint8_t* buf, size_t max, std::string& error) {
    if (fd_ == kNoPort) {
        error = "port closed";
        return -1;
    }
    DWORD n = 0;
    if (!::ReadFile(handle(fd_), buf, static_cast<DWORD>(max), &n, nullptr)) {
        const DWORD err = ::GetLastError();
        error = err == ERROR_ACCESS_DENIED || err == ERROR_BAD_COMMAND || err == ERROR_DEVICE_NOT_CONNECTED ||
                        err == ERROR_OPERATION_ABORTED
                    ? port_ + " disconnected"
                    : "read " + port_ + ": " + winError(err);
        return -1;
    }
    return static_cast<long>(n);
}

void SerialTransport::write(const uint8_t* data, size_t len) {
    if (fd_ == kNoPort) {
        return;
    }
    DWORD n = 0;
    ::WriteFile(handle(fd_), data, static_cast<DWORD>(len), &n, nullptr);  // a failure shows up on the read side
}

std::string SerialTransport::describe() const {
    return port_ + " · " + std::to_string(baud_);
}

namespace {

std::string narrow(const std::wstring& w) {
    if (w.empty()) {
        return {};
    }
    const int n = ::WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), nullptr, 0, nullptr, nullptr);
    std::string s(static_cast<size_t>(n), '\0');
    ::WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), s.data(), n, nullptr, nullptr);
    return s;
}

// USB vendor IDs of autopilot makers: ArduPilot (pid.codes), 3DR/PX4, CubePilot, Holybro, mRo, CUAV
bool autopilotVid(const std::string& hwid) {
    const std::string h = lower(hwid);
    for (const char* v : {"vid_1209", "vid_26ac", "vid_2dae", "vid_3162", "vid_35a7", "vid_3163"}) {
        if (h.find(v) != std::string::npos) {
            return true;
        }
    }
    return false;
}

std::vector<SerialPortInfo> platformPorts() {
    std::vector<SerialPortInfo> out;
    HDEVINFO set = ::SetupDiGetClassDevsW(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) {
        return out;
    }
    SP_DEVINFO_DATA dev{};
    dev.cbSize = sizeof(dev);
    for (DWORD i = 0; ::SetupDiEnumDeviceInfo(set, i, &dev); ++i) {
        // "COM4" lives in the device's registry key
        HKEY key = ::SetupDiOpenDevRegKey(set, &dev, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (key == INVALID_HANDLE_VALUE) {
            continue;
        }
        wchar_t name[64] = {};
        DWORD size = sizeof(name);
        DWORD type = 0;
        const LONG rc = ::RegQueryValueExW(key, L"PortName", nullptr, &type, reinterpret_cast<BYTE*>(name), &size);
        ::RegCloseKey(key);
        if (rc != ERROR_SUCCESS || type != REG_SZ || std::wstring(name).rfind(L"COM", 0) != 0) {
            continue;  // LPT and the like
        }
        SerialPortInfo p;
        p.path = narrow(name);
        // what the USB device calls itself ("Pixhawk6C"), else Windows' name ("USB Serial Device (COM4)")
        wchar_t text[256] = {};
        DEVPROPTYPE ptype = 0;
        std::string product;
        if (::SetupDiGetDevicePropertyW(set, &dev, &DEVPKEY_Device_BusReportedDeviceDesc, &ptype,
                                        reinterpret_cast<BYTE*>(text), sizeof(text), nullptr, 0) &&
            ptype == DEVPROP_TYPE_STRING) {
            product = narrow(text);
        }
        std::string friendly;
        if (::SetupDiGetDeviceRegistryPropertyW(set, &dev, SPDRP_FRIENDLYNAME, nullptr, reinterpret_cast<BYTE*>(text),
                                                sizeof(text), nullptr)) {
            friendly = narrow(text);
        }
        std::string hwid;
        if (::SetupDiGetDeviceRegistryPropertyW(set, &dev, SPDRP_HARDWAREID, nullptr, reinterpret_cast<BYTE*>(text),
                                                sizeof(text), nullptr)) {
            hwid = narrow(text);
        }
        p.label = !product.empty() ? product + " (" + p.path + ")" : (!friendly.empty() ? friendly : p.path);
        p.likely_autopilot = autopilotVid(hwid) || looksLikeAutopilot(p.label);
        out.push_back(std::move(p));
    }
    ::SetupDiDestroyDeviceInfoList(set);
    return out;
}

}  // namespace

#endif

std::vector<SerialPortInfo> listSerialPorts() {
    std::vector<SerialPortInfo> out = platformPorts();
    std::stable_sort(out.begin(), out.end(), [](const SerialPortInfo& a, const SerialPortInfo& b) {
        if (a.likely_autopilot != b.likely_autopilot) {
            return a.likely_autopilot;
        }
        return a.path < b.path;
    });
    return out;
}

}  // namespace gcs::link
