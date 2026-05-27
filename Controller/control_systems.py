def run_pid(target, current, dt, config, state, rate_input=None):
    """
    RUNPID  Generic PID Controller with Anti-Windup and Feed-Forward.
    -------------------------------------------------------------------------
    Purpose:
      Calculates the control output based on Proportional-Integral-Derivative
      logic. Supports "Derivative on Measurement" (to reduce kick) and 
      "Conditional Integration" (to prevent windup).
    """
    # --- 1. ERROR CALCULATION ---
    error = target - current
    
    # --- 2. PROPORTIONAL TERM (P) ---
    p_term = config['Kp'] * error
    
    # --- 3. DERIVATIVE TERM (D) ---
    # Strategy: Use specific rate if provided (Derivative on Measurement),
    # otherwise calculate rate from error (Derivative on Error).
    if rate_input is not None:
        # Mode: Derivative on Measurement (e.g., using Gyro 'q' for pitch damping)
        d_term = config['Kd'] * rate_input
    else:
        # Mode: Derivative on Error (Standard)
        error_rate = (error - state['PrevError']) / dt
        d_term = config['Kd'] * error_rate
        
    # --- 4. FEED-FORWARD / BASE VALUE ---
    # Adds a trim value (e.g., Throttle 0.7) if defined in Config.
    base_input = config.get('Base', 0.0)
    
    # --- 5. INTEGRAL LOGIC & ANTI-WINDUP (CONDITIONAL INTEGRATION) ---
    # Calculate a temporary output to check for saturation *before* integrating.
    current_i = config['Ki'] * state['Integrator']
    temp_output = base_input + p_term + current_i + d_term
    
    # Logic: Accumulate Integral ONLY if:
    #   a) The system is NOT saturated.
    #   b) The system IS saturated, but the Error helps to desaturate it.
    accumulate_integral = False
    
    if config['Min'] <= temp_output <= config['Max']:
        accumulate_integral = True  # Region: Linear (Safe)
    elif temp_output > config['Max'] and error < 0:
        accumulate_integral = True  # Region: Max Saturated, but Error is negative (Recovery)
    elif temp_output < config['Min'] and error > 0:
        accumulate_integral = True  # Region: Min Saturated, but Error is positive (Recovery)
        
    if accumulate_integral:
        state['Integrator'] += error * dt
        
    # --- 6. FINAL OUTPUT CALCULATION ---
    final_i = config['Ki'] * state['Integrator']
    raw_output = base_input + p_term + final_i + d_term
    
    # --- 7. SATURATION (ACTUATOR LIMITS) ---
    output = max(min(raw_output, config['Max']), config['Min'])
    
    # --- 8. STATE UPDATE ---
    state['PrevError'] = error
    
    return output, state