#include "preflight_runner.hpp"

#include "preflight_art.hpp"

#include "core/geo.hpp"
#include "core/plane_modes.hpp"
#include "link/mavlink_client.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QSettings>
#include <QStandardPaths>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace gcs::preflight;
using link::LinkEvent;

namespace {

constexpr float kLiftTestPct = 15.0f;   // just above spin-min, far below hover
constexpr float kLiftTestSecs = 2.0f;
constexpr int kPusherThrottle = 120;    // MANUAL_CONTROL z (0..1000): 12 %
constexpr double kPusherSecs = 2.0;
constexpr uint32_t kRcBit = 0x10000;
constexpr int kAccelSuccess = 16777215;
constexpr int kAccelFailed = 16777216;

double secs(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(b - a).count();
}

QString q(const std::string& s) {
    return QString::fromStdString(s);
}

// sweep: 0 -> +1 -> hold -> -1 -> hold -> 0 over 4.4 s
double sweepCmd(double t) {
    if (t < 0.6) return t / 0.6;
    if (t < 1.4) return 1.0;
    if (t < 2.6) return 1.0 - 2.0 * (t - 1.4) / 1.2;
    if (t < 3.4) return -1.0;
    if (t < 4.0) return -1.0 + (t - 3.4) / 0.6;
    return 0.0;
}
constexpr double kSweepSecs = 4.2;

int modePosition(int pwm) {
    if (pwm <= 900) return -1;
    if (pwm <= 1230) return 0;
    if (pwm <= 1360) return 1;
    if (pwm <= 1490) return 2;
    if (pwm <= 1620) return 3;
    if (pwm <= 1749) return 4;
    return 5;
}

}  // namespace

PreflightRunner::PreflightRunner(link::MavlinkClient* client) : client_(client) {
    frame_ = frameLayout(1, 1);
    loadProfile();
}

// ---- profile / report -----------------------------------------------------------------------

void PreflightRunner::loadProfile() {
    QSettings qs;
    profile_.set = qs.value(QStringLiteral("preflight/profile_set"), false).toBool();
    profile_.chemistry = static_cast<Chemistry>(qs.value(QStringLiteral("preflight/chemistry"), 0).toInt());
    profile_.cells = qs.value(QStringLiteral("preflight/cells"), 0).toInt();
    profile_.capacity_mah = qs.value(QStringLiteral("preflight/capacity_mah"), 0).toInt();
    profile_.endurance_min = qs.value(QStringLiteral("preflight/endurance_min"), 0.0).toDouble();
}

void PreflightRunner::saveProfile() const {
    QSettings qs;
    qs.setValue(QStringLiteral("preflight/profile_set"), profile_.set);
    qs.setValue(QStringLiteral("preflight/chemistry"), static_cast<int>(profile_.chemistry));
    qs.setValue(QStringLiteral("preflight/cells"), profile_.cells);
    qs.setValue(QStringLiteral("preflight/capacity_mah"), profile_.capacity_mah);
    qs.setValue(QStringLiteral("preflight/endurance_min"), profile_.endurance_min);
}

QString PreflightRunner::saveReport() const {
    const QString dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + QStringLiteral("/preflight");
    QDir().mkpath(dir);
    const QDateTime now = QDateTime::currentDateTime();
    const QString path = dir + QStringLiteral("/") + now.toString(QStringLiteral("yyyy-MM-dd_HHmmss")) +
                         (session_.turnaround() ? QStringLiteral("_turnaround") : QString()) + QStringLiteral(".json");
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return {};
    }
    f.write(QByteArray::fromStdString(session_.reportJson(now.toString(Qt::ISODate).toStdString(), s_, profile_)));
    return path;
}

// ---- lifecycle --------------------------------------------------------------------------------

void PreflightRunner::open(const VehicleState& s, bool turnaround) {
    s_ = s;
    open_ = true;
    if (!session_.active() || (session_.passed() && !session_.invalidReason(s, Clock::now()).empty()) ||
        session_.turnaround() != turnaround) {
        session_.begin(s, Clock::now(), turnaround);
        live_.clear();
        was_complete_ = false;
        motor_ok_.clear();
    }
    if (!fetched_) {
        auto names = paramsToFetch();
        names.push_back("MNT1_TYPE");
        client_->fetchParams(std::move(names));
        fetched_ = true;
    }
}

void PreflightRunner::close() {
    stopAll();
    open_ = false;
}

void PreflightRunner::noteFlight() {
    session_.noteFlight();
    passed_override_ = false;
}

bool PreflightRunner::launchAllowed(const VehicleState& s, Clock::time_point now) const {
    if (passed_override_) {
        return true;
    }
    return session_.passed() && session_.invalidReason(s, now).empty();
}

QString PreflightRunner::launchBlockReason(const VehicleState& s, Clock::time_point now) const {
    if (!session_.active()) {
        return QStringLiteral("Preflight not done: open PREFLIGHT (slot 3)");
    }
    const std::string why = session_.invalidReason(s, now);
    if (session_.passed()) {
        return QStringLiteral("Preflight no longer valid: %1. Open PREFLIGHT (slot 3)").arg(q(why));
    }
    return QStringLiteral("Preflight %1/%2: finish it in PREFLIGHT (slot 3)")
        .arg(session_.doneCount())
        .arg(session_.totalCount());
}

void PreflightRunner::overrideLaunch(Clock::time_point now) {
    session_.noteOverride("launch allowed without a complete preflight (" + std::to_string(session_.doneCount()) +
                              "/" + std::to_string(session_.totalCount()) + ")",
                          now);
    passed_override_ = true;
    saveReport();
}

// ---- helpers ----------------------------------------------------------------------------------

void PreflightRunner::setResult(const std::string& id, Status st, const std::string& detail, bool by_user,
                                const std::string& note) {
    session_.set(id, st, detail, by_user, now_, note);
}

void PreflightRunner::send(const std::string& tag, std::vector<link::CommandRequest> steps) {
    tag_done_[tag] = false;
    tag_result_[tag].clear();
    client_->runSequence("pf:" + tag, std::move(steps));
}

double PreflightRunner::outputNorm(int ch) const {
    if (ch < 1 || ch > 16 || !s_.has_servo_raw) {
        return std::nan("");
    }
    const std::string n = "SERVO" + std::to_string(ch);
    auto get = [&](const char* suf, float def) {
        const auto it = s_.params.find(n + suf);
        return it != s_.params.end() ? it->second : def;
    };
    const double mn = get("_MIN", 1100), mx = get("_MAX", 1900), tr = get("_TRIM", 1500);
    const double pwm = s_.servo_raw[ch - 1];
    if (pwm <= 0) {
        return std::nan("");
    }
    return pwm >= tr ? (pwm - tr) / std::max(1.0, mx - tr) : (pwm - tr) / std::max(1.0, tr - mn);
}

int PreflightRunner::rcChannel(const char* rcmap, int fallback) const {
    const auto it = s_.params.find(rcmap);
    return it != s_.params.end() && it->second >= 1 ? static_cast<int>(it->second) : fallback;
}

double PreflightRunner::rcNorm(int ch) const {
    if (ch < 1 || ch > 18 || s_.rc[ch - 1] == 0) {
        return 0.0;
    }
    return std::clamp((s_.rc[ch - 1] - 1500.0) / 400.0, -1.0, 1.0);
}

void PreflightRunner::saveMode() {
    if (!mode_saved_) {
        mode_before_ = s_.custom_mode;
        mode_saved_ = true;
    }
    restore_sent_ = false;
}

void PreflightRunner::restoreMode() {
    if (mode_saved_) {
        client_->setMode(mode_before_, "pf:restore");
        restore_sent_ = true;  // cleared once the vehicle is back in it (update)
    }
}

bool PreflightRunner::testRunning() const {
    return !running_.empty();
}

void PreflightRunner::stopAll() {
    if (running_.empty()) {
        return;
    }
    client_->stopManualControl();
    // stop motor tests and anything armed for a test
    std::vector<link::CommandRequest> stop;
    if (running_ == "mot.lift") {
        stop.push_back(link::makeMotorTest(1, 0.0f, 0.1f, 1));
    }
    if (s_.armed) {
        stop.push_back(link::makeArm(false, true));
    }
    if (!stop.empty()) {
        for (auto& c : stop) {
            c.optional = true;
        }
        client_->runSequence("pf:stop", std::move(stop));
    }
    if (running_ == "sens.compass" && compass_method_ == 2) {
        client_->runSequence("pf:stop", {link::makeCancelMagCal()});
    }
    restoreMode();
    running_.clear();
    phase_ = 0;
}

// ---- events --------------------------------------------------------------------------------------

void PreflightRunner::onEvent(const LinkEvent& ev) {
    if (ev.type == LinkEvent::Type::ParamSet) {
        const auto it = fix_from_to_.find(ev.tag);
        if (it != fix_from_to_.end()) {
            session_.noteParamWrite(ev.tag, it->second.first, it->second.second, ev.ok);
            if (onToast) {
                onToast(ev.ok ? QStringLiteral("%1 set to %2").arg(q(ev.tag)).arg(it->second.second)
                              : QStringLiteral("%1 not set: %2").arg(q(ev.tag), q(ev.text)),
                        ev.ok);
            }
            fix_from_to_.erase(it);
        }
        return;
    }
    if (ev.type != LinkEvent::Type::CommandResult || ev.tag.rfind("pf:", 0) != 0) {
        return;
    }
    const std::string tag = ev.tag.substr(3);
    tag_done_[tag] = true;
    tag_result_[tag] = ev.ok ? "ok" : (ev.text.empty() ? "failed" : ev.text);
}

// ---- status / text --------------------------------------------------------------------------------

Status PreflightRunner::status(const std::string& id) const {
    return session_.result(id).status;
}

QString PreflightRunner::detail(const std::string& id) const {
    return q(session_.result(id).detail);
}

QString PreflightRunner::instruction(const std::string& id) const {
    const StepDef* d = findStep(id);
    if (!d) {
        return {};
    }
    const bool run = running_ == id;
    if (id == "mot.lift" && motor_idx_ < static_cast<int>(motor_order_.size())) {
        const auto& m = frame_.motors[motor_order_[motor_idx_]];
        const QString which = QStringLiteral("motor %1 (%2, %3)")
                                  .arg(QChar(m.letter()))
                                  .arg(m.angle > 0 ? (std::abs(m.angle) < 90 ? QStringLiteral("front right") : QStringLiteral("back right"))
                                                   : (std::abs(m.angle) < 90 ? QStringLiteral("front left") : QStringLiteral("back left")))
                                  .arg(m.ccw ? QStringLiteral("counter-clockwise") : QStringLiteral("clockwise"));
        if (run && phase_ == 1) return QStringLiteral("Spinning %1 …").arg(which);
        if (run && phase_ == 2)
            return QStringLiteral("Did %1 spin, in the place shown, turning %2 seen from above?")
                .arg(which, m.ccw ? QStringLiteral("counter-clockwise") : QStringLiteral("clockwise"));
        return QStringLiteral("Next: %1. Hold SPIN to run it for 2 s at 15 %.").arg(which);
    }
    if (id == "mot.pusher" && run) {
        switch (phase_) {
        case 1: return QStringLiteral("Arming in MANUAL…");
        case 2: return QStringLiteral("Pusher spinning at 12 %…");
        case 3: return QStringLiteral("Disarming…");
        case 4: return QStringLiteral("Did the pusher spin and blow air backwards (away from the nose)?");
        default: break;
        }
    }
    if ((id == "surf.roll" || id == "surf.pitch" || id == "surf.yaw") && run) {
        if (phase_ == 1) return QStringLiteral("Switching to MANUAL (disarmed: no motor can spin)…");
        if (phase_ == 2) return QStringLiteral("Watch the surface: it must move the way the picture shows.");
        if (phase_ == 3) return QStringLiteral("Did it move the way shown, through its full throw, without binding?");
    }
    if (id == "surf.stabilize" && run) {
        if (phase_ == 1) return QStringLiteral("Switching to FBWA…");
        return QStringLiteral("Tilt the aircraft right wing down by hand: the right aileron must go DOWN and the left UP "
                              "(it fights the tilt).");
    }
    if (id == "link.rcfailsafe") {
        if (rcfs_phase_ == 0) return QStringLiteral("Turn the transmitter OFF now.");
        if (rcfs_phase_ == 1) return QStringLiteral("Failsafe seen. Turn the transmitter back ON.");
    }
    if (id == "sens.accel" && run && art_.pose > 0 && art_.pose <= 6) {
        return QStringLiteral("Place the aircraft %1, hold it still, then tap PLACED.").arg(art::poseName(art_.pose).toLower());
    }
    if (id == "sens.compass" && run && compass_method_ == 2) {
        return QStringLiteral("Rotate the aircraft slowly through every orientation: nose up, down, both sides, "
                              "upside down. The sphere fills in.");
    }
    if (id == "sens.compass" && compass_method_ == 1) {
        return QStringLiteral("Point the nose exactly at %1° true (north = 0). Use a known landmark or a phone "
                              "compass away from the aircraft, then calibrate.")
            .arg(fixed_yaw_deg_, 3, 10, QLatin1Char('0'));
    }
    return q(d->how);
}

// ---- buttons ----------------------------------------------------------------------------------------

std::vector<PreflightRunner::Button> PreflightRunner::buttons(const std::string& id) const {
    std::vector<Button> b;
    const StepDef* d = findStep(id);
    if (!d) {
        return b;
    }
    const Status st = status(id);
    const bool run = running_ == id;
    const bool busy = !running_.empty() && !run;
    const bool armed = s_.armed;
    const QString busy_reason = QStringLiteral("Another test is running");
    auto add = [&](std::string a, QString l, Style s = Style::Primary, bool hold = false, bool en = true,
                   QString why = {}) { b.push_back({std::move(a), std::move(l), s, hold, en, std::move(why)}); };
    auto skip = [&] {
        if (!d->critical && st != Status::Pass && st != Status::Skipped) {
            add("skip", QStringLiteral("SKIP"), Style::Neutral);
        }
    };
    auto confirmPair = [&](const QString& yes) {
        add("yes", yes, Style::Good);
        add("no", QStringLiteral("✗ NO"), Style::Bad);
    };

    if (id == "profile.battery") {
        add("chem:0", QStringLiteral("LiPo"), profile_.chemistry == Chemistry::LiPo ? Style::Good : Style::Neutral);
        add("chem:1", QStringLiteral("LiHV"), profile_.chemistry == Chemistry::LiHV ? Style::Good : Style::Neutral);
        add("chem:2", QStringLiteral("Li-ion"), profile_.chemistry == Chemistry::LiIon ? Style::Good : Style::Neutral);
        add("cells-", QStringLiteral("− CELL"), Style::Neutral);
        add("cells+", QStringLiteral("+ CELL"), Style::Neutral);
        add("cap-", QStringLiteral("− 500 mAh"), Style::Neutral);
        add("cap+", QStringLiteral("+ 500 mAh"), Style::Neutral);
        add("end-", QStringLiteral("− 1 min"), Style::Neutral);
        add("end+", QStringLiteral("+ 1 min"), Style::Neutral);
        add("save", QStringLiteral("✓ SAVE PROFILE"), Style::Good, false, profile_.cells > 0 && profile_.capacity_mah > 0,
            QStringLiteral("Set the cells and capacity"));
        return b;
    }

    if (d->kind == Kind::You) {
        if (id == "mot.gate") {
            add("props:off", QStringLiteral("PROPS OFF"), props_on_ || st != Status::Pass ? Style::Primary : Style::Good);
            add("props:on", QStringLiteral("PROPS ON · TIED DOWN, ALL CLEAR"), Style::Neutral);
            return b;
        }
        if (st != Status::Pass) {
            add("yes", QStringLiteral("✓ DONE"), Style::Good);
        } else {
            add("undo", QStringLiteral("UNDO"), Style::Neutral);
        }
        skip();
        return b;
    }

    if (d->kind == Kind::Fix) {
        const auto pend = fix_pending_.find(id);
        for (const auto& c : paramChecks()) {
            if (c.id != id) continue;
            const ParamVerdict v = checkParam(c, s_, profile_);
            if (pend != fix_pending_.end()) {
                add("fixconfirm", QStringLiteral("✓ WRITE %1 = %2").arg(q(c.name)).arg(v.recommended ? *v.recommended : 0.0f),
                    Style::Good);
                add("fixcancel", QStringLiteral("CANCEL"), Style::Neutral);
                return b;
            }
            if (v.known && !v.ok && v.recommended && !c.name.empty()) {
                add("fix", QStringLiteral("SET %1 → %2").arg(q(c.name)).arg(*v.recommended), Style::Primary, false, !armed,
                    QStringLiteral("Disarm first"));
            }
            if (v.known && !v.ok && !c.critical && st != Status::Skipped) {
                add("skip", QStringLiteral("KEEP AS IS"), Style::Neutral);
            }
        }
        return b;
    }

    if (d->kind == Kind::Auto) {
        if (id == "pay.video" && st != Status::Pass) {
            add("na", QStringLiteral("NO CAMERA"), Style::Neutral);
        } else if (id == "fin.ready") {
            add("start", QStringLiteral("PREPARE FOR LAUNCH"), Style::Primary, false, !busy && !armed,
                armed ? QStringLiteral("Disarmed only") : busy_reason);
        } else if (id == "fin.safety" && st != Status::Pass && st != Status::NotApplicable) {
            add("safetyoff", QStringLiteral("HOLD: SAFETY OFF"), Style::Danger, true, !armed, QStringLiteral("Disarmed only"));
        } else if (id == "sens.gps2" && st != Status::Pass) {
            add("na", QStringLiteral("NOT FITTED"), Style::Neutral);
        }
        if (st == Status::Fail || st == Status::Waiting || st == Status::Warn) {
            skip();
        }
        return b;
    }

    // ---- calibrations
    if (id == "sens.gyro" || id == "sens.level" || id == "sens.baro") {
        add("start", run ? QStringLiteral("CALIBRATING…") : QStringLiteral("CALIBRATE"), Style::Primary, false,
            !run && !busy && !armed, armed ? QStringLiteral("Disarmed only") : busy_reason);
        skip();
        return b;
    }
    if (id == "sens.accel") {
        if (run) {
            if (art_.pose >= 1 && art_.pose <= 6) {
                add("pose", QStringLiteral("✓ PLACED %1").arg(art::poseName(art_.pose)), Style::Good);
            }
            add("cancel", QStringLiteral("CANCEL"), Style::Neutral);
            return b;
        }
        add("already", QStringLiteral("✓ ALREADY CALIBRATED"), Style::Good, false, (s_.sensors_health & 0x2) != 0,
            QStringLiteral("The accelerometer reports unhealthy"));
        add("simple", QStringLiteral("SIMPLE CAL (LEVEL)"), Style::Primary, false, !busy && !armed);
        add("full", QStringLiteral("6-POSITION CAL"), Style::Primary, false, !busy && !armed);
        return b;
    }
    if (id == "sens.compass") {
        if (run && compass_method_ == 2) {
            add("cancel", QStringLiteral("CANCEL"), Style::Neutral);
            return b;
        }
        if (compass_method_ == 1 && !run) {
            add("yaw-10", QStringLiteral("−10°"), Style::Neutral);
            add("yaw-1", QStringLiteral("−1°"), Style::Neutral);
            add("yaw+1", QStringLiteral("+1°"), Style::Neutral);
            add("yaw+10", QStringLiteral("+10°"), Style::Neutral);
            add("fixedcal", QStringLiteral("CALIBRATE AT %1°").arg(fixed_yaw_deg_, 3, 10, QLatin1Char('0')),
                Style::Primary, false, !busy && !armed && s_.has_gps && s_.gps_fix >= 3,
                QStringLiteral("Needs a GPS fix (for declination)"));
            add("back", QStringLiteral("◀ METHODS"), Style::Neutral);
            return b;
        }
        if (st == Status::Pass && (tag_result_.count("compass") && tag_result_.at("compass") == "ok")) {
            add("reboot", QStringLiteral("REBOOT AUTOPILOT"), Style::Primary, true, !armed);
        }
        add("already", QStringLiteral("✓ ALREADY CALIBRATED"), Style::Good, false,
            (s_.sensors_health & 0x4) != 0 && s_.ekf_mag_var < 0.3,
            QStringLiteral("The compass isn't healthy or disagrees with the motion"));
        add("method:fixed", QStringLiteral("FROM A KNOWN HEADING"), Style::Primary, false, !busy && !armed);
        add("method:rotate", QStringLiteral("BY ROTATING IT"), Style::Primary, false, !busy && !armed);
        return b;
    }

    // ---- tests
    if (id == "link.sticks" || id == "sens.orientation") {
        add("reset", QStringLiteral("↻ START OVER"), Style::Neutral);
        return b;
    }
    if (id == "link.modes") {
        add("yes", QStringLiteral("✓ MODES ARE RIGHT"), Style::Good, false, art_.mode_positions_seen >= 2,
            QStringLiteral("Flip the switch through its positions first"));
        add("na", QStringLiteral("NO MODE SWITCH"), Style::Neutral);
        return b;
    }
    if (id == "link.rcfailsafe") {
        add("reset", QStringLiteral("↻ START OVER"), Style::Neutral);
        return b;
    }
    if (id == "sens.pitot") {
        add("reset", QStringLiteral("↻ START OVER"), Style::Neutral);
        return b;
    }
    if (id == "surf.roll" || id == "surf.pitch" || id == "surf.yaw" || id == "surf.stabilize") {
        if (run && phase_ == 3) {
            confirmPair(QStringLiteral("✓ MOVED AS SHOWN"));
            add("again", QStringLiteral("↻ AGAIN"), Style::Neutral);
        } else if (run && id == "surf.stabilize" && phase_ == 2) {
            confirmPair(QStringLiteral("✓ AILERONS PUSH BACK"));
        } else if (!run) {
            add("start", id == "surf.stabilize" ? QStringLiteral("START (FBWA)") : QStringLiteral("SWEEP"), Style::Primary,
                false, !busy && !armed, armed ? QStringLiteral("Disarmed only") : busy_reason);
            if (st == Status::Fail && id != "surf.stabilize") {
                add("reverse", QStringLiteral("REVERSE THIS SURFACE"), Style::Primary);
            }
        }
        if (id == "surf.yaw") skip();
        return b;
    }
    if (id == "mot.lift") {
        const bool safe = safetyState(s_) == SafetyState::Safe;
        const bool gate = status("mot.gate") == Status::Pass;
        if (run && phase_ == 2) {
            add("yes", QStringLiteral("✓ RIGHT MOTOR, RIGHT WAY"), Style::Good);
            add("wrongmotor", QStringLiteral("✗ WRONG MOTOR"), Style::Bad);
            add("wrongdir", QStringLiteral("✗ WRONG DIRECTION"), Style::Bad);
            add("again", QStringLiteral("↻ AGAIN"), Style::Neutral);
        } else if (!run || phase_ == 0) {
            const bool done = motor_idx_ >= static_cast<int>(motor_order_.size()) && !motor_order_.empty();
            if (!done) {
                const char letter = motor_order_.empty() ? 'A' : frame_.motors[motor_order_[std::min<size_t>(motor_idx_, motor_order_.size() - 1)]].letter();
                add("spin", QStringLiteral("HOLD: SPIN %1").arg(QChar(letter)), Style::Danger, true,
                    gate && !safe && !armed && !busy,
                    !gate ? QStringLiteral("Confirm the props choice first") : safe ? QStringLiteral("Safety is ON: hold SAFETY OFF") : armed ? QStringLiteral("Disarmed only") : busy_reason);
            } else {
                add("restart", QStringLiteral("↻ TEST ALL AGAIN"), Style::Neutral);
            }
            if (safe) add("safetyoff", QStringLiteral("HOLD: SAFETY OFF"), Style::Danger, true);
        }
        return b;
    }
    if (id == "mot.pusher") {
        const bool safe = safetyState(s_) == SafetyState::Safe;
        const bool gate = status("mot.gate") == Status::Pass;
        if (run && phase_ == 4) {
            confirmPair(QStringLiteral("✓ SPUN, AIR BACKWARDS"));
            add("again", QStringLiteral("↻ AGAIN"), Style::Neutral);
        } else if (!run) {
            add("spin", QStringLiteral("HOLD: SPIN PUSHER"), Style::Danger, true, gate && !safe && !armed && !busy,
                !gate ? QStringLiteral("Confirm the props choice first") : safe ? QStringLiteral("Safety is ON") : armed ? QStringLiteral("Disarmed only") : busy_reason);
            if (safe) add("safetyoff", QStringLiteral("HOLD: SAFETY OFF"), Style::Danger, true);
        }
        return b;
    }
    if (id == "pay.gimbal") {
        const auto mt = s_.params.find("MNT1_TYPE");
        if (run && phase_ == 2) {
            confirmPair(QStringLiteral("✓ MOVED AS SHOWN"));
        } else if (!run) {
            add("start", QStringLiteral("SWEEP GIMBAL"), Style::Primary, false, !busy && (mt == s_.params.end() || mt->second > 0),
                QStringLiteral("No gimbal configured (MNT1_TYPE = 0)"));
            add("na", QStringLiteral("NO GIMBAL"), Style::Neutral);
        }
        return b;
    }
    return b;
}

// ---- actions ------------------------------------------------------------------------------------------

void PreflightRunner::act(const std::string& id, const std::string& a) {
    const StepDef* d = findStep(id);
    if (!d) {
        return;
    }
    auto toast = [&](const QString& t, bool good) {
        if (onToast) onToast(t, good);
    };
    if (a == "skip") {
        setResult(id, Status::Skipped, live_.count(id) ? live_[id].detail : "", true, "skipped by the pilot");
        return;
    }
    if (a == "na") {
        setResult(id, Status::NotApplicable, "not fitted", true, "not fitted");
        return;
    }
    if (a == "undo") {
        setResult(id, Status::Pending, "", true);
        return;
    }
    if (a == "safetyoff") {
        if (onSafety) onSafety(false);
        return;
    }

    // profile
    if (id == "profile.battery") {
        if (a.rfind("chem:", 0) == 0) profile_.chemistry = static_cast<Chemistry>(std::stoi(a.substr(5)));
        if (a == "cells-") profile_.cells = std::max(1, profile_.cells - 1);
        if (a == "cells+") profile_.cells = std::min(24, profile_.cells + 1);
        if (a == "cap-") profile_.capacity_mah = std::max(0, profile_.capacity_mah - 500);
        if (a == "cap+") profile_.capacity_mah += 500;
        if (a == "end-") profile_.endurance_min = std::max(0.0, profile_.endurance_min - 1.0);
        if (a == "end+") profile_.endurance_min += 1.0;
        if (a == "save") {
            profile_.set = true;
            saveProfile();
            setResult(id, Status::Pass, std::to_string(profile_.cells) + "S " +
                                            (profile_.chemistry == Chemistry::LiPo ? "LiPo" : profile_.chemistry == Chemistry::LiHV ? "LiHV" : "Li-ion") +
                                            " · " + std::to_string(profile_.capacity_mah) + " mAh",
                      true);
        } else if (status(id) == Status::Pass) {
            setResult(id, Status::Pending, "", true);  // changed: save again
        }
        return;
    }

    // physical checks
    if (d->kind == Kind::You) {
        if (id == "mot.gate") {
            props_on_ = a == "props:on";
            setResult(id, Status::Pass, props_on_ ? "props ON, tied down, area clear" : "props OFF", true);
            return;
        }
        if (a == "yes") setResult(id, Status::Pass, "confirmed", true);
        return;
    }

    // parameter fixes
    if (d->kind == Kind::Fix) {
        for (const auto& c : paramChecks()) {
            if (c.id != id) continue;
            const ParamVerdict v = checkParam(c, s_, profile_);
            if (a == "fix") fix_pending_[id] = c.name;
            if (a == "fixcancel") fix_pending_.erase(id);
            if (a == "fixconfirm" && v.recommended) {
                fix_pending_.erase(id);
                fix_from_to_[c.name] = {v.current, *v.recommended};
                client_->setParam(c.name, *v.recommended);
            }
        }
        return;
    }

    if (id == "fin.ready" && a == "start") {
        send("final", {link::makeSetMode(plane::AUTO, "AUTO"), link::makeSetMissionCurrent(1, true),
                       link::makeRunPrearmChecks()});
        return;
    }

    // calibrations
    if ((id == "sens.gyro" || id == "sens.level" || id == "sens.baro") && a == "start") {
        running_ = id;
        phase_ = 1;
        phase_t_ = now_;
        send(id, {link::makeCalibration(id == "sens.gyro" ? link::Calibration::Gyro
                                         : id == "sens.level" ? link::Calibration::Level
                                                              : link::Calibration::BaroAirspeed)});
        return;
    }
    if (id == "sens.accel") {
        if (a == "already") setResult(id, Status::Pass, "calibrated earlier, accel healthy", true);
        if (a == "simple") {
            running_ = id;
            phase_ = 10;
            send("accel", {link::makeCalibration(link::Calibration::AccelSimple)});
        }
        if (a == "full") {
            running_ = id;
            phase_ = 1;
            phase_t_ = now_;
            send("accel", {link::makeCalibration(link::Calibration::AccelFull)});
        }
        if (a == "pose" && art_.pose >= 1 && art_.pose <= 6) {
            send("accelpos", {link::makeAccelCalPos(art_.pose)});
        }
        if (a == "cancel") stopAll();
        return;
    }
    if (id == "sens.compass") {
        if (a == "already") setResult(id, Status::Pass, "calibrated earlier, compass healthy and consistent", true);
        if (a == "method:fixed") {
            compass_method_ = 1;
            fixed_yaw_deg_ = static_cast<int>(std::lround(geo::wrap360(s_.yaw_deg)));
        }
        if (a == "back") compass_method_ = 0;
        if (a.rfind("yaw", 0) == 0) {
            fixed_yaw_deg_ = static_cast<int>(geo::wrap360(fixed_yaw_deg_ + std::stoi(a.substr(3))));
        }
        if (a == "fixedcal") {
            running_ = id;
            phase_ = 1;
            send("compass", {link::makeFixedMagCalYaw(static_cast<float>(fixed_yaw_deg_))});
        }
        if (a == "method:rotate") {
            compass_method_ = 2;
            running_ = id;
            phase_ = 1;
            phase_t_ = now_;
            send("compassstart", {link::makeStartMagCal()});
        }
        if (a == "cancel") {
            stopAll();
            compass_method_ = 0;
        }
        if (a == "reboot") {
            send("reboot", {link::makeReboot()});
            toast(QStringLiteral("Rebooting the autopilot: the link comes back in a few seconds"), true);
        }
        return;
    }

    // tests
    if (id == "link.sticks" && a == "reset") art_.stick_reached = {};
    if (id == "sens.orientation" && a == "reset") {
        art_.orient_roll = art_.orient_pitch = art_.orient_yaw = false;
        yaw0_ = s_.yaw_deg;
        setResult(id, Status::Pending, "", true);
    }
    if (id == "link.modes") {
        if (a == "yes") setResult(id, Status::Pass, std::to_string(art_.mode_positions_seen) + " positions checked", true);
    }
    if (id == "link.rcfailsafe" && a == "reset") {
        rcfs_phase_ = 0;
        setResult(id, Status::Pending, "", true);
    }
    if (id == "sens.pitot" && a == "reset") {
        pitot_peak_ = 0.0;
        setResult(id, Status::Pending, "", true);
    }

    if (id == "surf.roll" || id == "surf.pitch" || id == "surf.yaw" || id == "surf.stabilize") {
        if (a == "start" || a == "again") {
            running_ = id;
            phase_ = 1;
            phase_t_ = now_;
            saveMode();
            send("surfmode", {link::makeSetMode(id == "surf.stabilize" ? plane::FBWA : plane::MANUAL,
                                                id == "surf.stabilize" ? "FBWA" : "MANUAL")});
        }
        if (a == "yes" || a == "no") {
            const char* what = id == "surf.roll" ? "ailerons" : id == "surf.pitch" ? "elevator" : id == "surf.yaw" ? "rudder" : "stabilisation";
            setResult(id, a == "yes" ? Status::Pass : Status::Fail,
                      a == "yes" ? std::string(what) + " correct"
                                 : std::string(what) + " wrong: reverse the servo (button) or fix the linkage, then sweep again",
                      true);
            running_.clear();
            phase_ = 0;
            client_->stopManualControl();
            restoreMode();
        }
        if (a == "reverse") {
            const preflight::OutputMap m = outputMap(s_, static_cast<int>(frame_.motors.size()));
            std::vector<int> chans;
            if (id == "surf.roll") chans = m.ailerons;
            if (id == "surf.pitch" && m.elevator) chans = {m.elevator};
            if (id == "surf.yaw" && m.rudder) chans = {m.rudder};
            for (int ch : chans) {
                const std::string n = "SERVO" + std::to_string(ch) + "_REVERSED";
                const auto it = s_.params.find(n);
                const float from = it != s_.params.end() ? it->second : 0.0f;
                fix_from_to_[n] = {from, from > 0.5f ? 0.0f : 1.0f};
                client_->setParam(n, from > 0.5f ? 0.0f : 1.0f);
            }
            toast(chans.empty() ? QStringLiteral("No single servo to reverse (elevons / V-tail): fix in the mixer")
                                : QStringLiteral("Reversed; sweep again to check"),
                  !chans.empty());
            setResult(id, Status::Pending, "", true);
        }
        return;
    }

    if (id == "mot.lift") {
        if (a == "restart") {
            motor_idx_ = 0;
            motor_ok_.assign(motor_order_.size(), false);
            setResult(id, Status::Pending, "", true);
        }
        if (a == "spin" || a == "again") {
            if (motor_idx_ >= static_cast<int>(motor_order_.size())) return;
            const auto& m = frame_.motors[motor_order_[motor_idx_]];
            running_ = id;
            phase_ = 1;
            phase_t_ = now_;
            spun_channel_ = 0;
            send("motor", {link::makeMotorTest(m.test_order, kLiftTestPct, kLiftTestSecs)});
        }
        if (a == "yes") {
            motor_ok_.resize(motor_order_.size(), false);
            motor_ok_[motor_idx_] = true;
            ++motor_idx_;
            running_.clear();
            phase_ = 0;
            if (motor_idx_ >= static_cast<int>(motor_order_.size())) {
                setResult(id, Status::Pass, "all " + std::to_string(motor_order_.size()) + " lift motors: right place, right direction", true);
            }
        }
        if (a == "wrongmotor" || a == "wrongdir") {
            const auto& m = frame_.motors[motor_order_[motor_idx_]];
            setResult(id, Status::Fail,
                      a == "wrongmotor"
                          ? std::string("motor ") + m.letter() + " spun in the wrong place: swap the ESC signal leads, or fix SERVOn_FUNCTION for MOTOR" + std::to_string(m.number)
                          : std::string("motor ") + m.letter() + " spins the wrong way: swap any two of its three motor wires (or reverse it in the ESC)",
                      true);
            running_.clear();
            phase_ = 0;
        }
        return;
    }
    if (id == "mot.pusher") {
        if (a == "spin" || a == "again") {
            running_ = id;
            phase_ = 1;
            phase_t_ = now_;
            spun_channel_ = 0;
            saveMode();
            client_->manualControl(0, 0, 0, 0, 1500);  // throttle held at zero while arming
            send("pusherarm", {link::makeSetMode(plane::MANUAL, "MANUAL"), link::makeArm(true)});
        }
        if (a == "yes" || a == "no") {
            setResult(id, a == "yes" ? Status::Pass : Status::Fail,
                      a == "yes" ? "pusher spins and pushes" : "pusher wrong: if it blows forwards, swap two motor wires or flip the prop",
                      true);
            running_.clear();
            phase_ = 0;
            restoreMode();
        }
        return;
    }
    if (id == "pay.gimbal") {
        if (a == "start") {
            running_ = id;
            phase_ = 1;
            phase_t_ = now_;
        }
        if (a == "yes" || a == "no") {
            setResult(id, a == "yes" ? Status::Pass : Status::Fail, a == "yes" ? "gimbal moves as commanded" : "gimbal wrong", true);
            running_.clear();
            phase_ = 0;
            client_->runSequence("pf:gimbal", {link::makeMountControl(0.0f, 0.0f)});
        }
        return;
    }
}

// ---- per-frame -----------------------------------------------------------------------------------------

void PreflightRunner::update(const VehicleState& s, Clock::time_point now, double video_age_s) {
    s_ = s;
    now_ = now;
    video_age_ = video_age_s;

    // frame + motor order
    {
        const auto fc = s.params.find("Q_FRAME_CLASS"), ft = s.params.find("Q_FRAME_TYPE");
        if (fc != s.params.end() && ft != s.params.end()) {
            const FrameLayout f = frameLayout(static_cast<int>(fc->second), static_cast<int>(ft->second));
            if (f.name != frame_.name) {
                frame_ = f;
                motor_order_.clear();
            }
        }
        if (motor_order_.empty()) {
            for (size_t i = 0; i < frame_.motors.size(); ++i) motor_order_.push_back(static_cast<int>(i));
            std::sort(motor_order_.begin(), motor_order_.end(),
                      [&](int a, int b) { return frame_.motors[a].test_order < frame_.motors[b].test_order; });
            motor_ok_.assign(motor_order_.size(), false);
            motor_idx_ = 0;
        }
    }
    if (profile_.cells <= 0 && s.has_battery && s.battery_v > 0.0) {
        profile_.cells = guessCells(s.battery_v, profile_.chemistry);
    }

    if (!session_.active()) {
        return;
    }

    // automatic steps (and the automatic side of settings)
    for (const auto* d : session_.activeSteps()) {
        if (d->kind != Kind::Auto && d->kind != Kind::Fix) continue;
        const Status user = session_.result(d->id).status;
        if (user == Status::Skipped || user == Status::NotApplicable) {
            if (session_.result(d->id).by_user) continue;
        }
        std::optional<Eval> e;
        if (d->id == "pay.video") {
            e = video_age_ < 2.0 ? Eval{Status::Pass, "frames arriving"} : Eval{Status::Waiting, "no frames yet"};
        } else {
            e = evaluate(d->id, s, profile_, now);
        }
        if (e && d->id == "fin.ready" && e->status != Status::Pass && !prearm_.empty() && !prearmHealthy(s)) {
            for (const auto& r : prearm_) e->detail += "\n• " + r;
        }
        if (e) {
            live_[d->id] = *e;
            if (e->status == Status::Fail || e->status == Status::Waiting) {
                // a fixed problem comes back by itself; a non-critical one may be skipped
            }
            session_.set(d->id, e->status, e->detail, false, now);
        }
    }

    tickTests(now);
    if (mode_saved_ && restore_sent_ && running_.empty() && s.custom_mode == mode_before_) {
        mode_saved_ = restore_sent_ = false;  // back where we started
    }

    if (session_.complete() && !session_.passed()) {
        session_.markPassed(s, now);
        saveReport();
        if (onComplete) onComplete();
    }
    if (session_.passed() && !session_.invalidReason(s, now).empty() && !open_) {
        // stays recorded; the next open starts over
    }
}

void PreflightRunner::tickTests(Clock::time_point now) {
    const preflight::OutputMap m = outputMap(s_, static_cast<int>(frame_.motors.size()));

    // ---- always-on observers (they need no button) -------------------------------------------
    if (session_.includes("link.sticks") && status("link.sticks") != Status::Pass) {
        const int r = rcChannel("RCMAP_ROLL", 1), pi = rcChannel("RCMAP_PITCH", 2), t = rcChannel("RCMAP_THROTTLE", 3),
                  y = rcChannel("RCMAP_YAW", 4);
        auto mark = [&](int ch, int lo, int hi) {
            if (ch < 1 || ch > 18 || s_.rc[ch - 1] == 0) return;
            if (s_.rc[ch - 1] < 1150) art_.stick_reached[lo] = true;
            if (s_.rc[ch - 1] > 1850) art_.stick_reached[hi] = true;
        };
        mark(r, 0, 1);
        mark(pi, 2, 3);
        mark(t, 4, 5);
        mark(y, 6, 7);
        const bool all = std::all_of(art_.stick_reached.begin(), art_.stick_reached.end(), [](bool b) { return b; });
        const bool thr_low = t >= 1 && s_.rc[t - 1] > 0 && s_.rc[t - 1] < 1150;
        if (!s_.has_rc || s_.rc_count == 0) {
            setResult("link.sticks", Status::Waiting, "no RC input: is the transmitter on?", false);
        } else if (all && thr_low) {
            setResult("link.sticks", Status::Pass, "all four axes reach both ends; throttle low", false);
        } else {
            const int n = static_cast<int>(std::count(art_.stick_reached.begin(), art_.stick_reached.end(), true));
            setResult("link.sticks", Status::Waiting,
                      std::to_string(n) + "/8 stick ends reached" + (all && !thr_low ? "; now put the throttle low" : ""), false);
        }
    }
    if (session_.includes("link.modes")) {
        const int ch = rcChannel("FLTMODE_CH", 8);
        const int pos = ch >= 1 && ch <= 18 ? modePosition(s_.rc[ch - 1]) : -1;
        if (pos >= 0) mode_seen_[pos] = true;
        art_.mode_positions_seen = static_cast<int>(std::count(mode_seen_.begin(), mode_seen_.end(), true));
        art_.mode_live = pos;
        art_.mode_list.clear();
        for (int i = 0; i < 6; ++i) {
            const auto it = s_.params.find("FLTMODE" + std::to_string(i + 1));
            const char* name = it != s_.params.end() ? plane::modeName(static_cast<uint32_t>(it->second)) : nullptr;
            art_.mode_list.push_back(QStringLiteral("%1  %2%3")
                                         .arg(i + 1)
                                         .arg(name ? QString::fromLatin1(name) : QStringLiteral("?"))
                                         .arg(mode_seen_[i] ? QStringLiteral("  ✓") : QString()));
        }
    }
    if (session_.includes("link.rcfailsafe") && status("link.rcfailsafe") != Status::Pass) {
        const bool rc_bad = (s_.sensors_present & kRcBit) && !(s_.sensors_health & kRcBit);
        if (rcfs_phase_ == 0 && rc_bad) rcfs_phase_ = 1;
        if (rcfs_phase_ == 1 && !rc_bad && s_.rc_count > 0) {
            rcfs_phase_ = 2;
            setResult("link.rcfailsafe", Status::Pass, "failsafe triggered with the transmitter off, cleared when back on", false);
        } else if (rcfs_phase_ < 2) {
            setResult("link.rcfailsafe", Status::Waiting,
                      rcfs_phase_ == 0 ? "waiting for the RC failsafe…" : "failsafe seen; waiting for the transmitter", false);
        }
    }
    if (session_.includes("sens.pitot") && status("sens.pitot") != Status::Pass) {
        const auto t = s_.params.find("ARSPD_TYPE");
        if (t != s_.params.end() && t->second < 0.5f) {
            setResult("sens.pitot", Status::NotApplicable, "no airspeed sensor (ARSPD_TYPE = 0)", false);
        } else {
            pitot_peak_ = std::max(pitot_peak_, s_.airspeed);
            if (pitot_peak_ > 5.0 && s_.airspeed < 3.0) {
                setResult("sens.pitot", Status::Pass, "rose to " + std::to_string(static_cast<int>(pitot_peak_)) + " m/s and settled back", false);
            } else {
                setResult("sens.pitot", Status::Waiting,
                          pitot_peak_ > 5.0 ? "good rise; now let it settle" : "waiting for the airspeed to rise", false);
            }
        }
    }
    if (session_.includes("sens.orientation") && status("sens.orientation") != Status::Pass) {
        if (s_.roll_deg > 15) art_.orient_roll = true;
        if (s_.pitch_deg > 10) art_.orient_pitch = true;
        if (std::abs(geo::wrap180(s_.yaw_deg - yaw0_)) > 45) art_.orient_yaw = true;
        if (art_.orient_roll && art_.orient_pitch && art_.orient_yaw) {
            setResult("sens.orientation", Status::Pass, "roll, pitch and heading all follow", false);
        } else {
            setResult("sens.orientation", Status::Waiting,
                      std::string(art_.orient_roll ? "roll ✓ " : "roll · ") + (art_.orient_pitch ? "pitch ✓ " : "pitch · ") +
                          (art_.orient_yaw ? "heading ✓" : "heading ·"),
                      false);
        }
    }

    // ---- the test in progress ------------------------------------------------------------------
    const std::string id = running_;
    if (id.empty()) {
        art_.lit_motor = -1;
        art_.pusher_lit = false;
        art_.surf_cmd = 0.0;
        return;
    }
    const double t = secs(phase_t_, now);
    auto result = [&](const std::string& tag) -> std::string {
        const auto it = tag_done_.find(tag);
        return it != tag_done_.end() && it->second ? tag_result_[tag] : std::string();
    };

    if (id == "sens.gyro" || id == "sens.level" || id == "sens.baro") {
        const std::string r = result(id);
        if (!r.empty()) {
            const bool ok = r == "ok";
            std::string d = ok ? "calibrated" : "failed: " + r;
            Status st = ok ? Status::Pass : Status::Fail;
            if (ok && id == "sens.level" && (std::abs(s_.roll_deg) > 2.0 || std::abs(s_.pitch_deg) > 2.0)) {
                st = Status::Warn;
                d = "calibrated, but it still reads " + std::to_string(static_cast<int>(s_.roll_deg)) + "° / " +
                    std::to_string(static_cast<int>(s_.pitch_deg)) + "°: was it level?";
            }
            setResult(id, st, d, false);
            running_.clear();
        }
        return;
    }
    if (id == "sens.accel") {
        art_.pose = s_.accelcal_pos;
        if (phase_ == 10) {
            const std::string r = result("accel");
            if (!r.empty()) {
                setResult(id, r == "ok" ? Status::Pass : Status::Fail, r == "ok" ? "simple calibration done" : "failed: " + r, false);
                running_.clear();
            }
            return;
        }
        if (s_.accelcal_pos == kAccelSuccess) {
            setResult(id, Status::Pass, "6-position calibration done", false);
            running_.clear();
            art_.pose = 0;
        } else if (s_.accelcal_pos == kAccelFailed) {
            setResult(id, Status::Fail, "6-position calibration failed: hold it still in each pose and retry", false);
            running_.clear();
            art_.pose = 0;
        } else if (t > 600) {
            stopAll();
        }
        return;
    }
    if (id == "sens.compass") {
        if (compass_method_ == 1) {
            const std::string r = result("compass");
            if (!r.empty()) {
                setResult(id, r == "ok" ? Status::Pass : Status::Fail,
                          r == "ok" ? "calibrated at " + std::to_string(fixed_yaw_deg_) + "°; reboot to apply" : "failed: " + r, false);
                running_.clear();
                compass_method_ = 0;
            }
            return;
        }
        // onboard: wait for every active compass to report
        int active = 0, done = 0, ok = 0;
        double worst = 0.0;
        for (const auto& c : s_.magcal) {
            if (secs(c.rx, now) > 5.0 && !c.reported) continue;
            ++active;
            if (c.reported && c.rx >= phase_t_) {
                ++done;
                if (c.status == MAG_CAL_SUCCESS) ++ok;
                worst = std::max(worst, c.fitness);
            }
        }
        if (active > 0 && done == active) {
            tag_result_["compass"] = ok == done ? "ok" : "failed";
            setResult(id, ok == done ? Status::Pass : Status::Fail,
                      ok == done ? "fitness " + std::to_string(static_cast<int>(worst)) + " (lower is better), saved; reboot to apply"
                                 : "calibration failed: rotate more slowly and through every side",
                      false);
            running_.clear();
            compass_method_ = 0;
        } else if (t > 300) {
            stopAll();
            setResult(id, Status::Fail, "timed out", false);
        }
        return;
    }

    if (id == "surf.roll" || id == "surf.pitch" || id == "surf.yaw") {
        art_.axis = id == "surf.roll" ? 0 : id == "surf.pitch" ? 1 : 2;
        int ch = id == "surf.roll" ? m.aileron : id == "surf.pitch" ? m.elevator : m.rudder;
        if (!ch && id == "surf.pitch") ch = m.elevon_l;
        if (!ch && id == "surf.roll") ch = m.elevon_r;
        art_.surf_out = outputNorm(ch);
        if (phase_ == 1) {
            const std::string r = result("surfmode");
            if (r == "ok" && s_.custom_mode == plane::MANUAL) {
                phase_ = 2;
                phase_t_ = now;
            } else if (!r.empty() && r != "ok") {
                setResult(id, Status::Fail, "couldn't switch to MANUAL: " + r, false);
                running_.clear();
                restoreMode();
            }
            return;
        }
        if (phase_ == 2) {
            const double cmd = sweepCmd(t);
            art_.surf_cmd = cmd;
            const int v = static_cast<int>(std::lround(cmd * 1000.0));
            // MANUAL_CONTROL: y roll (+ right), x pitch (+ forward = nose down), r yaw (+ right)
            if (art_.axis == 0) client_->manualControl(0, v, 0, 0, 300);
            if (art_.axis == 1) client_->manualControl(-v, 0, 0, 0, 300);
            if (art_.axis == 2) client_->manualControl(0, 0, 0, v, 300);
            if (art_.axis == 0 && cmd > 0.95 && std::isfinite(art_.surf_out)) {
                surf_right_sign_ = art_.surf_out > 0 ? 1.0 : -1.0;
            }
            if (t > kSweepSecs) {
                phase_ = 3;
                client_->stopManualControl();
            }
        }
        return;
    }
    if (id == "surf.stabilize") {
        art_.axis = 0;
        art_.surf_out = outputNorm(m.aileron ? m.aileron : m.elevon_r);
        art_.surf_cmd = std::clamp(-s_.roll_deg / 30.0, -1.0, 1.0);  // what it should command against the tilt
        if (phase_ == 1) {
            const std::string r = result("surfmode");
            if (r == "ok" && s_.custom_mode == plane::FBWA) {
                phase_ = 2;
                phase_t_ = now;
            } else if (!r.empty() && r != "ok") {
                setResult(id, Status::Fail, "couldn't switch to FBWA: " + r, false);
                running_.clear();
                restoreMode();
            }
        }
        return;
    }

    if (id == "mot.lift") {
        if (motor_idx_ >= static_cast<int>(motor_order_.size())) {
            running_.clear();
            return;
        }
        const int mi = motor_order_[motor_idx_];
        art_.lit_motor = phase_ == 1 ? mi : -1;
        if (phase_ == 1) {
            const std::string r = result("motor");
            if (!r.empty() && r != "ok") {
                setResult(id, Status::Fail, "the vehicle refused the motor test: " + r, false);
                running_.clear();
                phase_ = 0;
                return;
            }
            for (int k = 0; k < static_cast<int>(m.lift.size()); ++k) {
                const int ch = m.lift[k];
                if (ch && s_.has_servo_raw && s_.servo_raw[ch - 1] > 1050) spun_channel_ = ch;
            }
            if (t > kLiftTestSecs + 0.6) {
                phase_ = 2;
                const int want = m.lift[frame_.motors[mi].number - 1];
                if (spun_channel_ && spun_channel_ != want) {
                    setResult(id, Status::Fail, "output " + std::to_string(spun_channel_) + " spun, expected " +
                                                    std::to_string(want) + ": check SERVOn_FUNCTION",
                              false);
                }
            }
        }
        return;
    }
    if (id == "mot.pusher") {
        art_.pusher_lit = phase_ == 2;
        if (phase_ == 1) {
            client_->manualControl(0, 0, 0, 0, 1500);
            const std::string r = result("pusherarm");
            if (r == "ok" && s_.armed) {
                phase_ = 2;
                phase_t_ = now;
            } else if (!r.empty() && r != "ok") {
                setResult(id, Status::Fail, "couldn't arm in MANUAL: " + r + " (see the PreArm message)", false);
                running_.clear();
                phase_ = 0;
                client_->stopManualControl();
                restoreMode();
            }
            return;
        }
        if (phase_ == 2) {
            client_->manualControl(0, 0, kPusherThrottle, 0, 300);
            if (m.throttle && s_.servo_raw[m.throttle - 1] > 1050) spun_channel_ = m.throttle;
            if (t > kPusherSecs) {
                phase_ = 3;
                phase_t_ = now;
                client_->stopManualControl();
                send("pusherdisarm", {link::makeArm(false, true)});
            }
            return;
        }
        if (phase_ == 3) {
            if (s_.armed && !result("pusherdisarm").empty() && t > 1.0) {
                send("pusherdisarm", {link::makeArm(false, true)});  // refused or lost: again until it's disarmed
                phase_t_ = now;
            }
            if (!s_.armed) {
                phase_ = 4;
                if (!spun_channel_) {
                    setResult(id, Status::Warn, "no throttle output seen: check the pusher ESC", false);
                }
            }
            return;
        }
        return;
    }
    if (id == "pay.gimbal") {
        struct P {
            double at, pitch, yaw;
        };
        static const P seq[] = {{0.0, -45, 0}, {1.6, 0, 40}, {3.2, 0, -40}, {4.8, 0, 0}};
        if (phase_ == 1) {
            for (int k = 3; k >= 0; --k) {
                if (t >= seq[k].at) {
                    if (art_.gimbal_pitch != seq[k].pitch || art_.gimbal_yaw != seq[k].yaw) {
                        art_.gimbal_pitch = seq[k].pitch;
                        art_.gimbal_yaw = seq[k].yaw;
                        client_->runSequence("pf:gimbal", {link::makeMountControl(static_cast<float>(seq[k].pitch),
                                                                                   static_cast<float>(seq[k].yaw))});
                    }
                    break;
                }
            }
            if (t > 6.0) phase_ = 2;
        }
        return;
    }
}

}  // namespace gcs::ui
