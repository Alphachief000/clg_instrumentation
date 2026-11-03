// ate_sequencer.cpp  -  Automated test sequencer & data logger (ATE-style)
// Build : g++ -std=c++17 -Wall -Wextra -o ate_sequencer ate_sequencer.cpp
// Run   : ./ate_sequencer --config limits.cfg --serial SN001
//         ./ate_sequencer --config limits.cfg --serial SN002 --fault
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// ---------- Data types ----------
struct TestStep {
    std::string name;   // also the instrument channel name
    std::string unit;
    double low;
    double high;
};

enum class Status { PASS, FAIL, ERROR };

const char* toString(Status s) {
    switch (s) {
        case Status::PASS:  return "PASS";
        case Status::FAIL:  return "FAIL";
        default:            return "ERROR";
    }
}

struct StepResult {
    TestStep step;
    double value;
    Status status;
    std::string note;
};

// ---------- Instrument abstraction ----------
// Today: simulated. Later: derive SerialInstrument to read a real Arduino/DMM.
class Instrument {
public:
    virtual ~Instrument() = default;
    virtual double read(const std::string& channel) = 0;
};

class SimulatedDut : public Instrument {
public:
    explicit SimulatedDut(bool injectFault, unsigned seed = 42)
        : fault_(injectFault), rng_(seed) {}

    double read(const std::string& channel) override {
        auto it = nominal_.find(channel);
        if (it == nominal_.end())
            throw std::runtime_error("Unknown channel: " + channel);

        double v = it->second;
        if (fault_ && channel == "5V_RAIL") v *= 0.82;   // injected fault: sagging rail
        if (fault_ && channel == "TEMP_C")  v = std::nan("");  // injected fault: dead sensor

        std::normal_distribution<double> noise(0.0, 0.005 * std::fabs(v));
        return v + noise(rng_);
    }

private:
    bool fault_;
    std::mt19937 rng_;
    std::map<std::string, double> nominal_{
        {"3V3_RAIL", 3.30}, {"5V_RAIL", 5.00}, {"CURRENT_MA", 120.0}, {"TEMP_C", 38.0}};
};

// ---------- Config loader ----------
std::vector<TestStep> loadConfig(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open config file: " + path);

    std::vector<TestStep> steps;
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string name, unit, lo, hi;
        if (!std::getline(ss, name, ',') || !std::getline(ss, unit, ',') ||
            !std::getline(ss, lo, ',')   || !std::getline(ss, hi, ','))
            throw std::runtime_error("Malformed config line " + std::to_string(lineNo));

        TestStep s{name, unit, std::stod(lo), std::stod(hi)};
        if (s.low >= s.high)
            throw std::runtime_error("Low limit >= high limit on line " + std::to_string(lineNo));
        steps.push_back(s);
    }
    if (steps.empty()) throw std::runtime_error("Config contains no test steps");
    return steps;
}

// ---------- Logging ----------
std::string timestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream os;
    os << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S");
    return os.str();
}

class CsvLogger {
public:
    explicit CsvLogger(const std::string& path) {
        bool fresh = !std::ifstream(path).good();
        out_.open(path, std::ios::app);
        if (!out_) throw std::runtime_error("Cannot open log file: " + path);
        if (fresh) out_ << "timestamp,serial,step,value,unit,low,high,status,note\n";
    }
    void log(const std::string& serial, const StepResult& r) {
        out_ << timestamp() << ',' << serial << ',' << r.step.name << ','
             << std::fixed << std::setprecision(3) << r.value << ',' << r.step.unit << ','
             << r.step.low << ',' << r.step.high << ',' << toString(r.status) << ','
             << r.note << '\n';
    }
private:
    std::ofstream out_;
};

// ---------- Test execution ----------
StepResult runStep(Instrument& dut, const TestStep& step, int samples = 10) {
    StepResult res{step, 0.0, Status::ERROR, ""};
    try {
        double sum = 0.0;
        for (int i = 0; i < samples; ++i) {
            double v = dut.read(step.name);
            if (std::isnan(v)) throw std::runtime_error("sensor returned NaN");
            sum += v;
        }
        res.value = sum / samples;                      // average to reduce noise
        res.status = (res.value >= step.low && res.value <= step.high) ? Status::PASS
                                                                       : Status::FAIL;
        if (res.status == Status::FAIL)
            res.note = res.value < step.low ? "below low limit" : "above high limit";
    } catch (const std::exception& e) {
        res.status = Status::ERROR;                     // measurement problem, not a product failure
        res.note = e.what();
    }
    return res;
}

int main(int argc, char* argv[]) {
    std::string configPath = "limits.cfg", serial = "SN000";
    bool fault = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) configPath = argv[++i];
        else if (a == "--serial" && i + 1 < argc) serial = argv[++i];
        else if (a == "--fault") fault = true;
        else { std::cerr << "Unknown argument: " << a << "\n"; return 2; }
    }

    try {
        auto steps = loadConfig(configPath);
        SimulatedDut dut(fault);
        CsvLogger logger("test_log.csv");

        std::cout << "=== Test run: " << serial << "  (" << timestamp() << ") ===\n";
        std::cout << std::left << std::setw(12) << "STEP" << std::setw(10) << "VALUE"
                  << std::setw(14) << "LIMITS" << std::setw(8) << "RESULT" << "NOTE\n";

        int pass = 0, fail = 0, err = 0;
        for (const auto& s : steps) {
            StepResult r = runStep(dut, s);
            logger.log(serial, r);

            std::ostringstream lim;
            lim << s.low << " - " << s.high;
            std::cout << std::left << std::setw(12) << s.name << std::fixed << std::setprecision(2)
                      << std::setw(10) << r.value << std::setw(14) << lim.str()
                      << std::setw(8) << toString(r.status) << r.note << "\n";

            if (r.status == Status::PASS) ++pass;
            else if (r.status == Status::FAIL) ++fail;
            else ++err;
        }

        bool ok = (fail == 0 && err == 0);
        std::cout << "Summary: " << pass << " pass, " << fail << " fail, " << err
                  << " error  ->  OVERALL " << (ok ? "PASS" : "FAIL") << "\n";
        return ok ? 0 : 1;                               // exit code usable by scripts / CI
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 2;
    }
}
