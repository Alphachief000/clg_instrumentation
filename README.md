# clg_instrumentation

## 1. Automated Test Sequencer & Data Logger (`ate_sequencer.cpp`)
Runs configurable test steps on a (simulated) device under test, checks readings against limits, logs to CSV, prints a pass/fail report.

    g++ -std=c++17 -Wall -Wextra -o ate_sequencer ate_sequencer.cpp
    ./ate_sequencer --config limits.cfg --serial SN001          # healthy unit
    ./ate_sequencer --config limits.cfg --serial SN002 --fault  # injected faults

Files: `limits.cfg` (test limits), `test_log.csv` (created on run).
Exit code: 0 = overall PASS, 1 = FAIL/ERROR, 2 = bad usage/config.

## 2. PID Voltage-Regulator Simulator (`pid_sim.cpp`)
Simulates an AVR-style PID loop on a two-lag plant, applies a load disturbance, measures overshoot, settling time, steady-state error and recovery time, and checks them against a spec.

    g++ -std=c++17 -Wall -Wextra -o pid_sim pid_sim.cpp
    ./pid_sim

Outputs `response_tuned.csv`, `response_aggressive.csv`, `response_sluggish.csv` (plot time vs output in Excel).
