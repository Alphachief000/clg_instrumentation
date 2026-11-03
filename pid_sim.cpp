// pid_sim.cpp  -  PID voltage-regulator simulation with automated performance tests
// Models the idea behind an excitation system's AVR: hold terminal voltage at a setpoint
// even when the load changes.
// Build : g++ -std=c++17 -Wall -Wextra -o pid_sim pid_sim.cpp
// Run   : ./pid_sim          (writes one CSV per gain set, e.g. response_tuned.csv)
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// ---------- PID controller ----------
class PID {
public:
    PID(double kp, double ki, double kd, double dt, double outMin, double outMax)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt), outMin_(outMin), outMax_(outMax) {}

    double update(double setpoint, double measurement) {
        double error = setpoint - measurement;

        // derivative on measurement (avoids "kick" when the setpoint steps)
        double dMeas = (measurement - prevMeas_) / dt_;
        prevMeas_ = measurement;

        double unsat = kp_ * error + integral_ - kd_ * dMeas;
        double out = std::clamp(unsat, outMin_, outMax_);

        // anti-windup: only integrate while output is not saturated (or error pulls it back)
        bool saturatedHigh = (unsat > outMax_ && error > 0);
        bool saturatedLow  = (unsat < outMin_ && error < 0);
        if (!saturatedHigh && !saturatedLow) integral_ += ki_ * error * dt_;

        return out;
    }

private:
    double kp_, ki_, kd_, dt_, outMin_, outMax_;
    double integral_ = 0.0, prevMeas_ = 0.0;
};

// ---------- Plant: exciter lag -> generator field lag (two first-order stages) ----------
class Plant {
public:
    Plant(double tauExciter, double tauField, double dt)
        : t1_(tauExciter), t2_(tauField), dt_(dt) {}

    double step(double control, double loadDisturbance) {
        x1_ += dt_ / t1_ * (-x1_ + control);        // exciter
        x2_ += dt_ / t2_ * (-x2_ + x1_);            // generator field
        return x2_ - loadDisturbance;               // terminal voltage drops under load
    }

private:
    double t1_, t2_, dt_;
    double x1_ = 0.0, x2_ = 0.0;
};

// ---------- Simulation & metrics ----------
struct Sample { double t, sp, y, u; };

struct Metrics {
    double overshootPct;
    double settlingTime;     // s, 2% band, after the setpoint step
    double steadyStateErr;   // % of setpoint, just before the load disturbance
    double recoveryTime;     // s, back inside 2% band after the load disturbance
};

struct Gains { std::string name; double kp, ki, kd; };

const double DT = 0.001;           // 1 ms time step
const double T_END = 12.0;
const double T_LOAD = 6.0;         // load disturbance applied here
const double LOAD_STEP = 0.10;     // 0.10 pu voltage drop
const double SETPOINT = 1.0;       // 1.0 per-unit voltage
const double BAND = 0.02;          // 2% settling band

std::vector<Sample> simulate(const Gains& g) {
    PID pid(g.kp, g.ki, g.kd, DT, 0.0, 10.0);       // exciter ceiling limits
    Plant plant(0.5, 1.0, DT);
    std::vector<Sample> log;
    double y = 0.0;
    for (double t = 0.0; t < T_END; t += DT) {
        double u = pid.update(SETPOINT, y);
        double d = (t >= T_LOAD) ? LOAD_STEP : 0.0;
        y = plant.step(u, d);
        if (!std::isfinite(y)) y = 1e9;              // flag numerical blow-up
        log.push_back({t, SETPOINT, y, u});
        if (std::fabs(y) > 1e6) break;               // diverged: stop early
    }
    return log;
}

Metrics measure(const std::vector<Sample>& log) {
    Metrics m{0, 0, 0, 0};
    double peak = 0.0;
    for (const auto& s : log) if (s.t < T_LOAD) peak = std::max(peak, s.y);
    m.overshootPct = std::max(0.0, (peak - SETPOINT) / SETPOINT * 100.0);

    // settling time: last instant outside the band before the load step
    double lastOut = 0.0;
    for (const auto& s : log)
        if (s.t < T_LOAD && std::fabs(s.y - SETPOINT) > BAND * SETPOINT) lastOut = s.t;
    m.settlingTime = lastOut;

    // steady-state error: mean |error| over the 1 s before the load step
    double sum = 0.0; int n = 0;
    for (const auto& s : log)
        if (s.t >= T_LOAD - 1.0 && s.t < T_LOAD) { sum += std::fabs(SETPOINT - s.y); ++n; }
    m.steadyStateErr = n ? sum / n / SETPOINT * 100.0 : 100.0;

    // recovery time: last instant outside the band after the load step
    double lastOutAfter = T_LOAD;
    for (const auto& s : log)
        if (s.t >= T_LOAD && std::fabs(s.y - SETPOINT) > BAND * SETPOINT) lastOutAfter = s.t;
    m.recoveryTime = lastOutAfter - T_LOAD;
    if (log.back().t < T_END - 0.01) m.recoveryTime = 1e9;   // diverged
    return m;
}

void writeCsv(const std::string& file, const std::vector<Sample>& log) {
    std::ofstream out(file);
    out << "time_s,setpoint,output,control\n";
    for (size_t i = 0; i < log.size(); i += 10)      // decimate to every 10 ms
        out << log[i].t << ',' << log[i].sp << ',' << log[i].y << ',' << log[i].u << '\n';
}

// ---------- Pass/fail specification ----------
struct Spec { double maxOvershoot = 10.0, maxSettling = 3.0, maxSSE = 1.0, maxRecovery = 3.0; };

bool check(const Metrics& m, const Spec& s, std::vector<std::string>& reasons) {
    if (m.overshootPct   > s.maxOvershoot) reasons.push_back("overshoot too high");
    if (m.settlingTime   > s.maxSettling)  reasons.push_back("settling too slow");
    if (m.steadyStateErr > s.maxSSE)       reasons.push_back("steady-state error too large");
    if (m.recoveryTime   > s.maxRecovery)  reasons.push_back("slow/no recovery after load step");
    return reasons.empty();
}

int main() {
    std::vector<Gains> cases = {
        {"tuned",      3.0,  1.5, 0.8},
        {"aggressive", 20.0, 40.0, 0.0},    // too much gain -> oscillation / instability
        {"sluggish",   0.5,  0.1, 0.0}      // too little gain -> slow response
    };
    Spec spec;

    std::cout << std::left << std::setw(12) << "CASE" << std::setw(12) << "OVERSHOOT%"
              << std::setw(11) << "SETTLE(s)" << std::setw(10) << "SSE%" << std::setw(12)
              << "RECOVER(s)" << "RESULT\n";

    int failures = 0;
    for (const auto& g : cases) {
        auto log = simulate(g);
        Metrics m = measure(log);
        writeCsv("response_" + g.name + ".csv", log);

        std::vector<std::string> why;
        bool ok = check(m, spec, why);
        if (!ok) ++failures;

        std::cout << std::left << std::fixed << std::setprecision(2) << std::setw(12) << g.name
                  << std::setw(12) << m.overshootPct << std::setw(11) << m.settlingTime
                  << std::setw(10) << m.steadyStateErr << std::setw(12)
                  << (m.recoveryTime > 1e6 ? 99.99 : m.recoveryTime) << (ok ? "PASS" : "FAIL");
        for (const auto& r : why) std::cout << "  [" << r << "]";
        std::cout << "\n";
    }
    return failures;   // non-zero when any case fails the spec
}
