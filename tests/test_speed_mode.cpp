#include "check.hpp"
#include "core/speed_mode.hpp"

#include <cstring>
#include <initializer_list>

using namespace gcs::speed;

static Speeds sitl() {
    Speeds v;
    v.loiter = 20.0;  // SITL quadplane: AIRSPEED_MIN 13, Q_ASSIST_SPEED 18 -> 20
    v.climb = 20.0;
    v.cruise = 22.0;
    v.max = 30.0;
    v.descent = 21.0;
    return v;
}

static void testTable() {
    const Speeds v = sitl();
    auto t = [&](Mode m, Situation s) { return command(m, s, v).target; };
    for (Mode m : {Mode::Eco, Mode::Normal, Mode::Fast}) {
        CHECK(t(m, Situation::None) == Target::Default);
        CHECK(command(m, Situation::None, v).airspeed == 0.0);
        CHECK(t(m, Situation::Landing) == Target::Cruise);
        CHECK(t(m, Situation::Climbing) == Target::Climb);  // FAST climbs at the best-climb speed too
        CHECK(command(m, Situation::Climbing, v).airspeed == 20.0);
    }
    // descents: the learned speed in ECO / NORMAL, MAX in FAST
    CHECK(t(Mode::Eco, Situation::Descending) == Target::Descent);
    CHECK(command(Mode::Normal, Situation::Descending, v).airspeed == 21.0);
    CHECK(t(Mode::Fast, Situation::Descending) == Target::Max);
    CHECK(command(Mode::Fast, Situation::Descending, v).airspeed == 30.0);
    Speeds unknown = v;
    unknown.descent = 0.0;
    CHECK(command(Mode::Eco, Situation::Descending, unknown).target == Target::Max);
    CHECK(command(Mode::Eco, Situation::Descending, v, 0.0, 20.0).airspeed == 25.0);  // the wind floor
    // on a circle: the learned speed in every mode (FAST too)
    for (Mode m : {Mode::Eco, Mode::Normal, Mode::Fast}) {
        CHECK(command(m, Situation::DescendingCircle, v).target == Target::Descent);
    }
    CHECK(command(Mode::Fast, Situation::DescendingCircle, unknown).target == Target::Cruise);
    CHECK(t(Mode::Eco, Situation::Circling) == Target::Loiter);
    CHECK(t(Mode::Normal, Situation::Circling) == Target::Cruise);
    CHECK(t(Mode::Fast, Situation::Circling) == Target::Cruise);
    CHECK(t(Mode::Eco, Situation::EnRoute) == Target::Cruise);  // no wind
    CHECK(t(Mode::Normal, Situation::EnRoute) == Target::Cruise);
    CHECK(t(Mode::Fast, Situation::EnRoute) == Target::Max);
}

static void testWind() {
    const Speeds v = sitl();
    // ECO en route: + half the headwind, - half a tailwind, within LOITER..MAX
    Command c = command(Mode::Eco, Situation::EnRoute, v, 6.0, 6.0);
    CHECK(c.target == Target::CruiseUp);
    CHECK_NEAR(c.airspeed, 25.0, 1e-9);
    c = command(Mode::Eco, Situation::EnRoute, v, 30.0, 0.0);
    CHECK_NEAR(c.airspeed, 30.0, 1e-9);  // clamped at MAX
    c = command(Mode::Eco, Situation::EnRoute, v, -3.0, 0.0);
    CHECK(c.target == Target::CruiseDown);
    CHECK_NEAR(c.airspeed, 20.5, 1e-9);
    c = command(Mode::Eco, Situation::EnRoute, v, -10.0, 0.0);
    CHECK_NEAR(c.airspeed, 20.0, 1e-9);  // never below LOITER
    c = command(Mode::Eco, Situation::EnRoute, v, 0.6, 0.0);
    CHECK(c.target == Target::Cruise);  // a small change is still CRUISE
    // NORMAL / FAST don't follow the wind en route
    CHECK_NEAR(command(Mode::Normal, Situation::EnRoute, v, 10.0, 10.0).airspeed, 22.0, 1e-9);
    // the wind floor: never slower than wind + 5, capped at MAX
    c = command(Mode::Eco, Situation::Circling, v, 0.0, 18.0);
    CHECK(c.target == Target::Wind);
    CHECK_NEAR(c.airspeed, 23.0, 1e-9);
    c = command(Mode::Normal, Situation::Climbing, v, 0.0, 40.0);
    CHECK_NEAR(c.airspeed, 30.0, 1e-9);
    c = command(Mode::Eco, Situation::Circling, v, 0.0, 10.0);
    CHECK(c.target == Target::Loiter);  // 20 > 15: untouched
    // headwind component
    CHECK_NEAR(headwind(270.0, 10.0, 270.0), 10.0, 1e-9);  // flying west into a west wind
    CHECK_NEAR(headwind(270.0, 10.0, 90.0), -10.0, 1e-9);  // flying east: tailwind
    CHECK_NEAR(headwind(270.0, 10.0, 0.0), 0.0, 1e-9);     // crosswind
}

static void testSpeeds() {
    CHECK(loiterSpeed(13.0, 25.0, 18.0) == 20.0);  // SITL's quadplane: assist above min
    CHECK(loiterSpeed(16.0, 25.0, 12.0) == 16.0);
    CHECK(loiterSpeed(16.0, 25.0, 0.0) == 16.0);   // Q_ASSIST off
    CHECK(loiterSpeed(13.0, 18.0, 18.0) == 18.0);  // never faster than cruise
    CHECK(std::strcmp(targetName(Target::Climb), "CLIMB") == 0);
    CHECK(std::strcmp(targetName(Target::Default), "") == 0);
    // the chip: only LOITER / CRUISE / MAX
    const Speeds v = sitl();
    CHECK(chipTarget(command(Mode::Normal, Situation::Climbing, v), v) == Target::Loiter);
    CHECK(chipTarget(command(Mode::Eco, Situation::EnRoute, v, 6.0, 6.0), v) == Target::Cruise);   // 25
    CHECK(chipTarget(command(Mode::Eco, Situation::EnRoute, v, 30.0, 0.0), v) == Target::Max);     // 30
    CHECK(chipTarget(command(Mode::Eco, Situation::Descending, v), v) == Target::Cruise);  // 21: a tie reads CRUISE
    Speeds slow = v;
    slow.descent = 20.4;
    CHECK(chipTarget(command(Mode::Eco, Situation::Descending, slow), slow) == Target::Loiter);
    CHECK(chipTarget(command(Mode::Eco, Situation::Circling, v, 0.0, 18.0), v) == Target::Cruise); // WIND 23
    CHECK(chipTarget(command(Mode::Eco, Situation::None, v), v) == Target::Default);
}

int main() {
    testTable();
    testWind();
    testSpeeds();
    return TEST_MAIN_RESULT();
}
