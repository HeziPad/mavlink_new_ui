#include "preflight_runner.hpp"

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
constexpr double kMotorGapSecs = 0.8;   // between two motors
constexpr int kPusherThrottle = 120;    // MANUAL_CONTROL z (0..1000): 12 %
constexpr double kPusherSecs = 2.0;
constexpr double kSwingNeeded = 0.3;    // of the output's travel, each way (elevon mixing halves it)
constexpr double kAmpsRise = 0.2;       // A: a spinning motor draws at least this
constexpr int kSpinPwm = 1050;          // an output above this is spinning

double secs(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(b - a).count();
}

QString q(const std::string& s) {
    return QString::fromStdString(s);
}

// sweep: 0 -> +1 -> hold -> -1 -> hold -> 0 over 4.2 s
double sweepCmd(double t) {
    if (t < 0.6) return t / 0.6;
    if (t < 1.4) return 1.0;
    if (t < 2.6) return 1.0 - 2.0 * (t - 1.4) / 1.2;
    if (t < 3.4) return -1.0;
    if (t < 4.0) return -1.0 + (t - 3.4) / 0.6;
    return 0.0;
}
constexpr double kSweepSecs = 4.2;

std::string where(const LiftMotor& m) {
    const bool right = m.angle > 0;
    const bool front = std::abs(m.angle) < 90;
    return std::string(front ? "front " : "back ") + (right ? "right" : "left");
}

}  // namespace

PreflightRunner::PreflightRunner(link::MavlinkClient* client) : client_(client) {
    frame_ = frameLayout(1, 1);
    loadProfile();
}

// ---- profile / report -----------------------------------------------------------------------

void PreflightRunner::loadProfile() {
    QSettings qs;
    profile_.chemistry = static_cast<Chemistry>(std::clamp(qs.value(QStringLiteral("preflight/chemistry"), 0).toInt(), 0, 2));
    profile_.cells = qs.value(QStringLiteral("preflight/cells"), 0).toInt();
    profile_.capacity_mah = qs.value(QStringLiteral("preflight/capacity_mah"), 0).toInt();
    profile_.endurance_min = qs.value(QStringLiteral("preflight/endurance_min"), 0.0).toDouble();
}

void PreflightRunner::saveProfile() const {
    QSettings qs;
    qs.setValue(QStringLiteral("preflight/chemistry"), static_cast<int>(profile_.chemistry));
    qs.setValue(QStringLiteral("preflight/cells"), profile_.cells);
    qs.setValue(QStringLiteral("preflight/capacity_mah"), profile_.capacity_mah);
    qs.setValue(QStringLiteral("preflight/endurance_min"), profile_.endurance_min);
}

void PreflightRunner::setBattery(Chemistry chem, int capacity_mah, double endurance_min) {
    if (chem != profile_.chemistry) {
        profile_.cells = 0;  // read off the pack again for this chemistry
    }
    profile_.chemistry = chem;
    profile_.capacity_mah = std::max(0, capacity_mah);
    profile_.endurance_min = std::max(0.0, endurance_min);
    saveProfile();
}

QString PreflightRunner::saveReport() const {
    const QString dir = QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).filePath(QStringLiteral("preflight"));
    QDir().mkpath(dir);
    const QDateTime now = QDateTime::currentDateTime();
    const QString path = QDir(dir).filePath(now.toString(QStringLiteral("yyyy-MM-dd_HHmmss")) +
                                            (session_.turnaround() ? QStringLiteral("_turnaround") : QString()) +
                                            QStringLiteral(".json"));
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return {};
    }
    f.write(QByteArray::fromStdString(session_.reportJson(now.toString(Qt::ISODate).toStdString(), s_, profile_)));
    return path;
}

// ---- lifecycle --------------------------------------------------------------------------------

void PreflightRunner::newSession(bool turnaround) {
    stopAll();
    session_.begin(s_, Clock::now(), turnaround);
    flow_.start(session_, Clock::now());
    motor_idx_ = 0;
    motor_faults_.clear();
    motor_notes_.clear();
    pitot_peak_ = 0.0;
    started_row_.clear();
}

void PreflightRunner::open(const VehicleState& s, bool turnaround) {
    s_ = s;
    open_ = true;
    if (!session_.active() || (session_.passed() && !session_.invalidReason(s, Clock::now()).empty()) ||
        session_.turnaround() != turnaround) {
        newSession(turnaround);
    } else if (session_.passed() && flow_.phase() != Phase::Result) {
        flow_.showResult();
    }
    if (!fetched_) {
        client_->fetchParams(paramsToFetch());
        fetched_ = true;
    }
}

void PreflightRunner::restart() {
    newSession(session_.turnaround());
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
        return QStringLiteral("Preflight not done: tap PREFLIGHT");
    }
    const std::string why = session_.invalidReason(s, now);
    if (session_.passed()) {
        return QStringLiteral("Preflight no longer valid: %1. Tap PREFLIGHT").arg(q(why));
    }
    const size_t n = problems(session_).size();
    if (flow_.phase() == Phase::Result && n > 0) {
        return QStringLiteral("Preflight found %1 problem%2: tap PREFLIGHT").arg(n).arg(n == 1 ? "" : "s");
    }
    return QStringLiteral("Preflight not finished: tap PREFLIGHT");
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

void PreflightRunner::restoreSafety() {
    if (safety_restore_ && !s_.armed) {
        safety_restore_ = false;
        if (onSafety) onSafety(true);
    }
}

void PreflightRunner::abortTest(const std::string& why) {
    if (running_.empty()) {
        return;
    }
    client_->stopManualControl();
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
    if (running_ == "pay.gimbal") {
        client_->runSequence("pf:gimbal", {link::makeMountControl(0.0f, 0.0f)});
    }
    const std::string was = running_;
    running_.clear();
    phase_ = 0;
    setResult(was, Status::Fail, why, false, why);
    restoreMode();
    art_ = Art{};
}

void PreflightRunner::stopAll() {
    if (cal_ != Cal::None) {
        cancelCal();
    }
    if (running_.empty()) {
        restoreSafety();
        return;
    }
    // the test running and every moving part still to come: stopped
    const std::string was = running_;
    abortTest("stopped");
    if (findStep(was) && findStep(was)->stage == Stage::Moving) {
        for (const auto* d : session_.activeSteps()) {
            if (d->stage == Stage::Moving && status(d->id) == Status::Pending) {
                setResult(d->id, Status::Fail, "stopped", true, "stopped");
            }
        }
    }
    restoreSafety();
}

// ---- events --------------------------------------------------------------------------------------

void PreflightRunner::onEvent(const LinkEvent& ev) {
    if (ev.type == LinkEvent::Type::ParamSet) {
        const auto it = fix_from_to_.find(ev.tag);
        if (it == fix_from_to_.end()) {
            return;
        }
        session_.noteParamWrite(ev.tag, it->second.first, it->second.second, ev.ok);
        for (const auto& c : paramChecks()) {
            if (c.name == ev.tag && ev.ok) {
                setup_done_.push_back(c.title);
                if (c.reboot) {
                    reboot_pending_ = true;
                    reboot_boot_count_ = s_.boot_count;
                }
            }
        }
        setup_failed_ |= !ev.ok;
        fix_from_to_.erase(it);
        setup_pending_.erase(ev.tag);
        if (setup_pending_.empty() && onToast) {
            QString list;
            for (const auto& t : setup_done_) {
                list += (list.isEmpty() ? QString() : QStringLiteral(" · ")) + q(t);
            }
            onToast(setup_failed_ ? QStringLiteral("Some settings couldn't be written: try SET IT UP again")
                                  : QStringLiteral("Fixed: %1").arg(list),
                    !setup_failed_);
            setup_done_.clear();
            setup_failed_ = false;
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

// ---- what the operator does -------------------------------------------------------------------------

void PreflightRunner::setLook(const std::string& id, bool done) {
    const StepDef* d = findStep(id);
    if (d && d->stage == Stage::Look && d->kind == Kind::You) {
        setResult(id, done ? Status::Pass : Status::Pending, done ? "checked" : "", true);
    }
}

void PreflightRunner::lookContinue() {
    if (lookComplete(session_)) {
        flow_.lookDone(now_);
    }
}

void PreflightRunner::goMoving() {
    if (flow_.phase() != Phase::Gate || s_.armed) {
        return;
    }
    if (safetyState(s_) == SafetyState::Safe) {
        safety_restore_ = true;
        if (onSafety) onSafety(false);
    }
    flow_.gateGo(now_);
}

void PreflightRunner::rerun() {
    stopAll();
    for (const auto* d : session_.activeSteps()) {
        if (d->kind == Kind::Test && d->stage != Stage::Look && status(d->id) != Status::Pass) {
            setResult(d->id, Status::Pending, "");
        }
    }
    motor_idx_ = 0;
    motor_faults_.clear();
    motor_notes_.clear();
    started_row_.clear();
    flow_.rerun(now_);
}

bool PreflightRunner::setupFixable() const {
    return !s_.armed && setupReport(s_, profile_).fixable();
}

void PreflightRunner::fixSetup() {
    if (s_.armed || !setup_pending_.empty()) {
        return;
    }
    for (const auto& w : setupReport(s_, profile_).wrong) {
        if (!w.verdict.recommended || w.check->name.empty()) continue;
        fix_from_to_[w.check->name] = {w.verdict.current, *w.verdict.recommended};
        setup_pending_.insert(w.check->name);
        client_->setParam(w.check->name, *w.verdict.recommended);
    }
}

void PreflightRunner::reboot() {
    if (s_.armed) {
        return;
    }
    send("reboot", {link::makeReboot()});
    if (onToast) onToast(QStringLiteral("Restarting the autopilot: the link comes back in a few seconds"), true);
}

// ---- calibrations ---------------------------------------------------------------------------------------

PreflightRunner::Cal PreflightRunner::calFor(const std::string& id) const {
    auto bad = [&](uint32_t bit) {
        return (s_.sensors_present & bit) && (s_.sensors_enabled & bit) && !(s_.sensors_health & bit);
    };
    if (id == "sens.health" || id == "sens.ekf") {
        if (bad(0x4) || bad(0x80000) || (s_.has_ekf && s_.ekf_mag_var >= kEkfVarCaution)) return Cal::Compass;
        if (id == "sens.health" && (bad(0x2) || bad(0x40000))) return Cal::Accel;
    }
    return Cal::None;
}

void PreflightRunner::startCal(Cal c) {
    if (c == Cal::None || cal_ != Cal::None || !running_.empty() || s_.armed) {
        return;
    }
    cal_ = c;
    cal_t_ = now_;
    if (c == Cal::Compass) {
        send("compassstart", {link::makeStartMagCal()});
    } else {
        send("accel", {link::makeCalibration(link::Calibration::AccelSimple)});
    }
}

void PreflightRunner::cancelCal() {
    if (cal_ == Cal::Compass) {
        client_->runSequence("pf:stop", {link::makeCancelMagCal()});
    }
    cal_ = Cal::None;
}

QString PreflightRunner::calText() const {
    if (cal_ == Cal::Compass) {
        return QStringLiteral("Pick the aircraft up and turn it slowly through every direction: nose up, nose down, "
                              "each wing down, upside down. The sphere fills in as it learns.");
    }
    if (cal_ == Cal::Accel) {
        return QStringLiteral("Keep the aircraft level and still…");
    }
    return {};
}

int PreflightRunner::calPercent() const {
    int pct = 0;
    for (const auto& c : s_.magcal) {
        if (c.active) pct = std::max(pct, c.pct);
    }
    return pct;
}

void PreflightRunner::tickCal(Clock::time_point now) {
    if (cal_ == Cal::Accel) {
        const auto it = tag_done_.find("accel");
        if (it != tag_done_.end() && it->second) {
            const bool ok = tag_result_["accel"] == "ok";
            if (onToast) onToast(ok ? QStringLiteral("Accelerometer calibrated") : QStringLiteral("Accelerometer calibration failed: keep it still and try again"), ok);
            cal_ = Cal::None;
        }
        return;
    }
    if (cal_ != Cal::Compass) {
        return;
    }
    // onboard: wait for every compass that is calibrating to report
    int active = 0, done = 0, ok = 0;
    for (const auto& c : s_.magcal) {
        if (secs(c.rx, now) > 5.0 && !c.reported) continue;
        ++active;
        if (c.reported && c.rx >= cal_t_) {
            ++done;
            if (c.status == MAG_CAL_SUCCESS) ++ok;
        }
    }
    if (active > 0 && done == active) {
        const bool good = ok == done;
        if (good) {
            reboot_pending_ = true;
            reboot_boot_count_ = s_.boot_count;
        }
        if (onToast) onToast(good ? QStringLiteral("Compass calibrated: restart the autopilot to use it")
                                  : QStringLiteral("Compass calibration failed: turn it more slowly, through every side"),
                             good);
        cal_ = Cal::None;
    } else if (secs(cal_t_, now) > 300) {
        cancelCal();
        if (onToast) onToast(QStringLiteral("Compass calibration timed out"), false);
    }
}

// ---- tests -----------------------------------------------------------------------------------------------

std::vector<int> PreflightRunner::surfaceChannels(const std::string& id) const {
    const OutputMap m = outputMap(s_, static_cast<int>(frame_.motors.size()));
    std::vector<int> v;
    auto add = [&](int ch) {
        if (ch && std::find(v.begin(), v.end(), ch) == v.end()) v.push_back(ch);
    };
    if (id == "surf.roll") {
        for (int ch : m.ailerons) add(ch);
        add(m.elevon_l);
        add(m.elevon_r);
    } else if (id == "surf.pitch") {
        add(m.elevator);
        add(m.elevon_l);
        add(m.elevon_r);
        add(m.vtail_l);
        add(m.vtail_r);
    } else if (id == "surf.yaw") {
        add(m.rudder);
        add(m.vtail_l);
        add(m.vtail_r);
    }
    return v;
}

void PreflightRunner::startTest(const std::string& id) {
    const StepDef* d = findStep(id);
    if (!d || !running_.empty() || cal_ != Cal::None) {
        return;
    }
    if (s_.armed) {
        setResult(id, Status::Fail, "the aircraft is armed: tests run disarmed only");
        return;
    }
    if (d->stage == Stage::Moving && safetyState(s_) == SafetyState::Safe) {
        setResult(id, Status::Waiting, "turning safety off");
        return;  // tried again next frame
    }
    phase_t_ = now_;
    if (id == "surf.roll" || id == "surf.pitch" || id == "surf.yaw") {
        if (surfaceChannels(id).empty()) {
            setResult(id, Status::NotApplicable, id == "surf.yaw" ? "no rudder" : "no output for this surface");
            return;
        }
        running_ = id;
        phase_ = 1;
        swing_.clear();
        saveMode();
        send("surfmode", {link::makeSetMode(plane::MANUAL, "MANUAL")});
        setResult(id, Status::Waiting, "switching to MANUAL (disarmed: nothing spins)");
        return;
    }
    if (id == "mot.lift") {
        if (motor_order_.empty()) {
            setResult(id, Status::Fail, "the frame has no lift motors");
            return;
        }
        running_ = id;
        motor_idx_ = 0;
        motor_faults_.clear();
        motor_notes_.clear();
        phase_ = 3;  // starts with the gap, then the first motor
        return;
    }
    if (id == "mot.pusher") {
        running_ = id;
        phase_ = 1;
        spun_channel_ = 0;
        amps0_ = std::max(0.0, s_.battery_a);
        amps_peak_ = amps0_;
        saveMode();
        client_->manualControl(0, 0, 0, 0, 1500);  // throttle held at zero while arming
        send("pusherarm", {link::makeSetMode(plane::MANUAL, "MANUAL"), link::makeArm(true)});
        setResult(id, Status::Waiting, "arming in MANUAL");
        return;
    }
    if (id == "pay.gimbal") {
        const auto mt = s_.params.find("MNT1_TYPE");
        if (mt == s_.params.end() || mt->second < 0.5f) {
            setResult(id, Status::NotApplicable, "no gimbal");
            return;
        }
        running_ = id;
        phase_ = 1;
        gimbal_seen_ = false;
        gimbal_min_pitch_ = 0.0;
        gimbal_yaw_lo_ = gimbal_yaw_hi_ = 0.0;
        setResult(id, Status::Waiting, "moving the gimbal");
        return;
    }
}

void PreflightRunner::endTest(const std::string& id, Status st, const std::string& detail) {
    setResult(id, st, detail);
    running_.clear();
    phase_ = 0;
    art_ = Art{};
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
        }
    }
    // the cell count, read off the pack: a battery that reports its cells says
    // so; otherwise the voltage. A flat pack can look like one cell fewer, so
    // a guess only ever raises what was seen before.
    if (s.has_battery && s.battery_v > 0.0 && !s.armed) {
        const int cells = s.cell_v.size() >= 2 ? static_cast<int>(s.cell_v.size()) : guessCells(s.battery_v, profile_.chemistry);
        if (cells > 0 && (s.cell_v.size() >= 2 ? cells != profile_.cells : cells > profile_.cells)) {
            profile_.cells = cells;
            saveProfile();
        }
    }
    if (reboot_pending_ && s.boot_count != reboot_boot_count_) {
        reboot_pending_ = false;  // restarted
    }

    if (!session_.active()) {
        return;
    }

    // automatic steps
    for (const auto* d : session_.activeSteps()) {
        if (d->kind != Kind::Auto) continue;
        std::optional<Eval> e;
        if (d->id == "pay.video") {
            e = video_age_ < 2.0 ? Eval{Status::Pass, "pictures arriving"} : Eval{Status::Waiting, "no pictures yet"};
        } else {
            e = evaluate(d->id, s, profile_, now);
        }
        if (!e) continue;
        if (d->id == "fin.ready" && e->status != Status::Pass && !prearmHealthy(s)) {
            // With safety ON the autopilot's own check fails on that alone;
            // Launch turns safety off, so that one doesn't count here.
            const bool only_safety =
                std::all_of(prearm_.begin(), prearm_.end(),
                            [](const std::string& r) { return r.find("afety") != std::string::npos; });
            if (safetyState(s) == SafetyState::Safe && only_safety && !s.armed && gpsHealthy(s) && ekfHealthy(s)) {
                e = Eval{Status::Pass, "ready (Launch turns safety off)"};
            } else {
                for (const auto& r : prearm_) e->detail += "\n• " + r;
            }
        }
        std::string note;
        if (flow_.timedOut(d->id) && !isDone(e->status) && e->status != Status::Fail) {
            // waited long enough: a required step fails, an optional one is a note
            e->status = d->critical ? Status::Fail : Status::Warn;
            note = "timed out";
        }
        session_.set(d->id, e->status, e->detail, false, now, note);
    }

    // the pitot on the look card: an observer, no button
    if (session_.includes("sens.pitot") && status("sens.pitot") != Status::Pass) {
        const auto t = s_.params.find("ARSPD_TYPE");
        if (t != s_.params.end() && t->second < 0.5f) {
            setResult("sens.pitot", Status::NotApplicable, "no airspeed sensor");
        } else {
            pitot_peak_ = std::max(pitot_peak_, s_.airspeed);
            if (pitot_peak_ > 5.0 && s_.airspeed < 3.0) {
                setResult("sens.pitot", Status::Pass,
                          "rose to " + std::to_string(static_cast<int>(pitot_peak_)) + " m/s and settled back");
            } else {
                setResult("sens.pitot", Status::Waiting,
                          pitot_peak_ > 5.0 ? "good, now let it settle" : "waiting for the airspeed to rise");
            }
        }
    }

    if (open_) {
        flow_.tick(session_, now);
        const std::string row = flow_.currentId();
        const StepDef* d = row.empty() ? nullptr : findStep(row);
        if (flow_.phase() == Phase::Scan && d) {
            if (row != started_row_) {
                started_row_ = row;
                if (row == "fin.ready") {
                    send("prearm", {link::makeRunPrearmChecks()});  // its reasons arrive sooner
                }
            }
            if (d->kind == Kind::Test && running_.empty() && !isDone(status(row)) && status(row) != Status::Fail) {
                startTest(row);
            }
        }
        // a test whose row timed out (the flow has moved on)
        if (!running_.empty() && flow_.timedOut(running_)) {
            abortTest("timed out");
        }
        // after the moving parts: safety back as it was
        if (safety_restore_ && running_.empty() && (!d || d->stage != Stage::Moving)) {
            restoreSafety();
        }
    }

    tickTests(now);
    tickCal(now);
    if (mode_saved_ && restore_sent_ && running_.empty() && s.custom_mode == mode_before_) {
        mode_saved_ = restore_sent_ = false;  // back where we started
    }

    if (session_.complete() && !session_.passed()) {
        session_.markPassed(s, now);
        saveReport();
        if (onComplete) onComplete();
    }
}

void PreflightRunner::tickTests(Clock::time_point now) {
    const std::string id = running_;
    if (id.empty()) {
        return;
    }
    const OutputMap m = outputMap(s_, static_cast<int>(frame_.motors.size()));
    const double t = secs(phase_t_, now);
    auto result = [&](const std::string& tag) -> std::string {
        const auto it = tag_done_.find(tag);
        return it != tag_done_.end() && it->second ? tag_result_[tag] : std::string();
    };
    // a current sensor that reads high at rest (power.current's note) can't tell a spin
    const bool amps_known = s_.has_battery && s_.battery_a >= 0.0 && amps0_ < 8.0;

    if (id == "surf.roll" || id == "surf.pitch" || id == "surf.yaw") {
        art_.axis = id == "surf.roll" ? 0 : id == "surf.pitch" ? 1 : 2;
        const std::vector<int> chans = surfaceChannels(id);
        art_.surf_out = chans.empty() ? std::nan("") : outputNorm(chans.front());
        if (phase_ == 1) {
            const std::string r = result("surfmode");
            if (r == "ok" && s_.custom_mode == plane::MANUAL) {
                phase_ = 2;
                phase_t_ = now;
                setResult(id, Status::Waiting, "moving it right, then left");
            } else if (!r.empty() && r != "ok") {
                endTest(id, Status::Fail, "couldn't switch to MANUAL: " + r);
                restoreMode();
            }
            return;
        }
        const double cmd = sweepCmd(t);
        art_.surf_cmd = cmd;
        const int v = static_cast<int>(std::lround(cmd * 1000.0));
        // MANUAL_CONTROL: y roll (+ right), x pitch (+ forward = nose down), r yaw (+ right)
        if (art_.axis == 0) client_->manualControl(0, v, 0, 0, 300);
        if (art_.axis == 1) client_->manualControl(-v, 0, 0, 0, 300);
        if (art_.axis == 2) client_->manualControl(0, 0, 0, v, 300);
        for (int ch : chans) {
            const double o = outputNorm(ch);
            if (!std::isfinite(o)) continue;
            auto it = swing_.find(ch);
            if (it == swing_.end()) {
                swing_[ch] = {o, o};
            } else {
                it->second.first = std::min(it->second.first, o);
                it->second.second = std::max(it->second.second, o);
            }
        }
        if (t <= kSweepSecs) {
            return;
        }
        client_->stopManualControl();
        restoreMode();
        if (!s_.has_servo_raw) {
            endTest(id, Status::Warn, "the autopilot doesn't report its outputs: watch the surfaces move yourself");
            return;
        }
        std::string stuck;
        for (int ch : chans) {
            const auto it = swing_.find(ch);
            const bool moved = it != swing_.end() && it->second.first <= -kSwingNeeded && it->second.second >= kSwingNeeded;
            if (!moved) {
                stuck += (stuck.empty() ? "output " : ", ") + std::to_string(ch);
            }
        }
        endTest(id, stuck.empty() ? Status::Pass : Status::Fail,
                stuck.empty() ? "moved both ways" : stuck + " didn't move both ways");
        return;
    }

    if (id == "mot.lift") {
        if (phase_ == 3) {
            // the gap before the next motor (and before the first)
            art_.lit_motor = -1;
            if (t < kMotorGapSecs) {
                return;
            }
            if (motor_idx_ >= static_cast<int>(motor_order_.size())) {
                std::string d;
                for (const auto& f : motor_faults_) d += (d.empty() ? "" : " · ") + f;
                if (!d.empty()) {
                    endTest(id, Status::Fail, d);
                } else {
                    for (const auto& f : motor_notes_) d += (d.empty() ? "" : " · ") + f;
                    endTest(id, d.empty() ? Status::Pass : Status::Warn,
                            d.empty() ? "all " + std::to_string(motor_order_.size()) + " spun, each on its own output" : d);
                }
                return;
            }
            const auto& mo = frame_.motors[motor_order_[motor_idx_]];
            phase_ = 1;
            phase_t_ = now;
            spun_channel_ = 0;
            amps0_ = std::max(0.0, s_.battery_a);
            amps_peak_ = amps0_;
            send("motor", {link::makeMotorTest(mo.test_order, kLiftTestPct, kLiftTestSecs)});
            setResult(id, Status::Waiting,
                      std::string("motor ") + mo.letter() + " (" + where(mo) + "), " + std::to_string(motor_idx_ + 1) +
                          " of " + std::to_string(motor_order_.size()));
            return;
        }
        const int mi = motor_order_[motor_idx_];
        const auto& mo = frame_.motors[mi];
        art_.lit_motor = mi;
        const std::string r = result("motor");
        if (!r.empty() && r != "ok") {
            endTest(id, Status::Fail, "the autopilot refused the motor test: " + r);
            return;
        }
        for (int ch : m.lift) {
            if (ch && s_.has_servo_raw && s_.servo_raw[ch - 1] > kSpinPwm) spun_channel_ = ch;
        }
        amps_peak_ = std::max(amps_peak_, s_.battery_a);
        if (t < kLiftTestSecs + 0.6) {
            return;
        }
        const int want = mo.number >= 1 && mo.number <= static_cast<int>(m.lift.size()) ? m.lift[mo.number - 1] : 0;
        const std::string name = std::string("motor ") + mo.letter();
        if (!s_.has_servo_raw) {
            motor_notes_.push_back(name + ": outputs not reported");
        } else if (!spun_channel_) {
            motor_faults_.push_back(name + " didn't spin");
        } else if (spun_channel_ != want) {
            motor_faults_.push_back(name + " came out on output " + std::to_string(spun_channel_) + ", not " +
                                    std::to_string(want));
        } else if (amps_known && amps_peak_ - amps0_ < kAmpsRise) {
            motor_notes_.push_back(name + " drew no current: is it connected?");
        }
        ++motor_idx_;
        phase_ = 3;
        phase_t_ = now;
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
                setResult(id, Status::Waiting, "spinning at 12 % for 2 s");
            } else if (!r.empty() && r != "ok") {
                client_->stopManualControl();
                restoreMode();
                endTest(id, Status::Fail, "couldn't arm to spin it: " + r);
            }
            return;
        }
        if (phase_ == 2) {
            client_->manualControl(0, 0, kPusherThrottle, 0, 300);
            if (m.throttle && s_.has_servo_raw && s_.servo_raw[m.throttle - 1] > kSpinPwm) spun_channel_ = m.throttle;
            amps_peak_ = std::max(amps_peak_, s_.battery_a);
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
                restoreMode();
                if (!s_.has_servo_raw) {
                    endTest(id, Status::Warn, "the autopilot doesn't report its outputs");
                } else if (!spun_channel_) {
                    endTest(id, Status::Fail, "the pusher output didn't come on");
                } else if (amps_known && amps_peak_ - amps0_ < kAmpsRise) {
                    endTest(id, Status::Warn, "the output came on but it drew no current: is it connected?");
                } else {
                    endTest(id, Status::Pass, "spun");
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
        if (s_.has_gimbal && secs(s_.gimbal_rx, now) < 1.0) {
            const double yaw = s_.gimbal_yaw_earth ? geo::wrap180(s_.gimbal_yaw - s_.yaw_deg) : s_.gimbal_yaw;
            if (!gimbal_seen_) {
                gimbal_yaw_lo_ = gimbal_yaw_hi_ = yaw;
                gimbal_seen_ = true;
            }
            gimbal_min_pitch_ = std::min(gimbal_min_pitch_, s_.gimbal_pitch);
            gimbal_yaw_lo_ = std::min(gimbal_yaw_lo_, yaw);
            gimbal_yaw_hi_ = std::max(gimbal_yaw_hi_, yaw);
        }
        if (t > 6.4) {
            if (!gimbal_seen_) {
                endTest(id, Status::Warn, "the gimbal doesn't report where it points");
            } else if (gimbal_min_pitch_ < -30.0 && gimbal_yaw_hi_ - gimbal_yaw_lo_ > 40.0) {
                endTest(id, Status::Pass, "followed down, right and left");
            } else {
                endTest(id, Status::Fail, "it didn't follow the commands");
            }
        }
        return;
    }
}

}  // namespace gcs::ui
