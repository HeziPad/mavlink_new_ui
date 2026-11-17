#pragma once

// Tiny tween helpers driven by the 60 Hz UI tick. No QPropertyAnimation: every
// animated value is a plain member stepped in tick(dt), so a widget repaints
// only while something it owns is actually moving.

#include <algorithm>
#include <cmath>

namespace gcs::ui {

inline double easeOutCubic(double t) {
    t = std::clamp(t, 0.0, 1.0);
    const double u = 1.0 - t;
    return 1.0 - u * u * u;
}

inline double easeInOutCubic(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t < 0.5 ? 4.0 * t * t * t : 1.0 - std::pow(-2.0 * t + 2.0, 3.0) / 2.0;
}

inline double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

// Exponential approach to a target; first set() snaps.
struct Tween {
    double value = 0.0;
    double target = 0.0;
    double rate = 12.0;  // 1/s
    bool primed = false;

    void set(double t) {
        target = t;
        if (!primed) {
            value = t;
            primed = true;
        }
    }
    void snap(double t) {
        value = target = t;
        primed = true;
    }
    // Returns true while still moving.
    bool step(double dt) {
        if (value == target) {
            return false;
        }
        const double k = 1.0 - std::exp(-rate * dt);
        value += (target - value) * k;
        if (std::abs(target - value) < 1e-3) {
            value = target;
        }
        return true;
    }
};

// Same, for angles in degrees (shortest way round).
struct AngleTween {
    double value = 0.0;
    double target = 0.0;
    double rate = 14.0;
    bool primed = false;

    void set(double t) {
        target = t;
        if (!primed) {
            value = t;
            primed = true;
        }
    }
    bool step(double dt) {
        double d = std::fmod(target - value + 540.0, 360.0) - 180.0;
        if (std::abs(d) < 1e-3) {
            value = target;
            return false;
        }
        value += d * (1.0 - std::exp(-rate * dt));
        value = std::fmod(value + 360.0, 360.0);
        return true;
    }
};

// One-shot timeline 0 -> 1 over `duration` seconds; inactive when < 0.
struct Pulse {
    double t = -1.0;
    double duration = 1.0;

    void start(double dur) {
        duration = dur;
        t = 0.0;
    }
    bool active() const { return t >= 0.0 && t < 1.0; }
    double progress() const { return std::clamp(t, 0.0, 1.0); }
    bool step(double dt) {
        if (!active()) {
            return false;
        }
        t += dt / duration;
        if (t >= 1.0) {
            t = -1.0;
        }
        return true;
    }
};

}  // namespace gcs::ui
