# Flight Control Systems: `control_systems.py`

This module contains the primary digital control logic for the T-38 Talon's autopilot system. The `run_pid` function is a highly robust, discrete-time Proportional-Integral-Derivative (PID) controller designed specifically to handle the non-linear dynamics of atmospheric flight.

## 🎯 The Core Advantage: Feed-Forward Control

A standard feedback loop relies exclusively on *error* (the difference between target and current state) to generate a control output. In aviation, this is fundamentally flawed for steady-state flight. 

If the T-38 requires 85% throttle to maintain Mach 0.8 at 15,000 meters, a pure PID controller must accumulate a massive Integral error just to output that 85%. This reliance on the Integrator causes extreme lag when engaging the autopilot and guarantees severe overshoot during altitude or speed changes.

**The Solution:** This controller implements a **Feed-Forward Architecture (`Base`)**. 
Rather than forcing the PID to "discover" the trim state, the main loop queries the pre-calculated `t38_control_matrix.pkl` to find the exact required throttle and elevator deflection for the current flight envelope. This baseline value is injected directly into the controller output. 

$$Output = U_{base} + K_p e + K_i \int e \, dt + K_d \dot{x}$$

**Why it matters:** By handling the steady-state trim via Feed-Forward ($U_{base}$), the PID controller is freed to do what it does best: reject transient disturbances (like sudden altitude drops or G-load shifts) and smoothly close the final few percentages of error.

---

## 🛠️ Advanced Controller Features

Beyond standard PID logic, this function incorporates two critical safeguards required for flight software:

### 1. Anti-Windup (Conditional Integration)
When an aircraft's actuators reach their physical limits (e.g., the elevator is pegged at 15 degrees), standard controllers continue to accumulate Integral error if the target state hasn't been reached. This "windup" causes the aircraft to aggressively overshoot the target once it finally regains control authority.
* **The Fix:** The algorithm uses Conditional Integration. It calculates a speculative `temp_output`. If the system is saturated (hitting `Min` or `Max` limits), the Integrator is instantly frozen. It only resumes accumulating if the error reverses direction and actively helps desaturate the actuator.

### 2. Derivative on Measurement (Anti-Kick)
Standard derivative logic calculates the rate of change of the *error*. If a pilot inputs a sudden, step-change command (e.g., commanding a new altitude), the error derivative spikes to near-infinity, causing the controller to violently "kick" the actuators.
* **The Fix:** The `run_pid` function accepts an optional `rate_input`. For pitch control, instead of deriving the pitch error, the controller is fed the raw pitch rate ($q$) directly from the gyroscope data. This provides smooth, continuous damping without the mathematical instability of a step-input kick.

---

## 📥 Inputs & 📤 Outputs

**Function Signature:**
`run_pid(target, current, dt, config, state, rate_input=None)`

**Inputs:**
* `target`: The desired setpoint (e.g., Mach 0.8).
* `current`: The current state from the physics engine.
* `dt`: Time step ($0.01$ seconds).
* `config`: A dictionary containing gains (`Kp`, `Ki`, `Kd`), actuator limits (`Min`, `Max`), and the Feed-Forward trim (`Base`).
* `state`: A dictionary storing `Integrator` accumulation and `PrevError` across simulation frames.
* `rate_input`: Optional direct measurement rate (like $q$) to bypass error-derivative calculation.

**Outputs:**
* `output`: The clamped, safe actuator command (e.g., throttle percentage or elevator radians).
* `state`: The updated internal memory dictionary to be passed into the next frame.

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>
