import math
import numpy as np

def flight_dynamics(state, ac, env, controls):
    """
    FLIGHT DYNAMICS ENGINE (T-38 TALON)
    Description: 3-DOF Longitudinal Physics Engine.
                 Calculates Aerodynamics, Propulsion, Ground Physics,
                 and State Derivatives for Time Integration.
    """
    # =========================================================================
    # 1. INITIALIZATION & UNPACKING
    # =========================================================================
    # --- Aircraft Geometry & Mass ---
    S_ref   = ac['Geo']['S_ref']
    c_bar   = ac['Geo']['c_bar']
    I_yy    = ac['Mass']['Iyy']
    epsilon = ac['Engine']['epsilon'] # Thrust inclination angle [rad]
    
    # --- State Vectors ---
    x_E   = state['x_E']
    z_E   = state['z_E']     # Note: z_E is positive DOWN
    h     = -z_E             # Altitude (positive UP)
    u     = state['u']
    w     = state['w']
    theta = state['theta']
    q     = state['q']
    m     = state['m']
    
    # --- Environment ---
    rho = env['rho']
    a   = env['a']
    g   = env['g']
    mu  = env['mu'] # Friction Coefficient
    
    # --- Aerodynamic Derivatives ---
    CL_delta = ac['Aero']['CL_delta']
    Cm0      = ac['Aero']['Cm0']
    Cma      = ac['Aero']['Cm_alpha']
    Cmd      = ac['Aero']['Cm_delta']
    
    # --- Control Inputs ---
    # Python dictionary .get() method replaces MATLAB's isfield()
    # It attempts to grab the value, and defaults to 0.0 if the key doesn't exist
    delta_e          = controls.get('ElevatorDeflection', 0.0)
    throttle_setting = controls.get('ThrottleSetting', 0.0)
    d_sb             = controls.get('SpeedBrake', 0.0)
    d_gear           = controls.get('Gear', 0.0)

    # =========================================================================
    # 2. KINEMATICS
    # =========================================================================
    # Velocity & Air Data
    V = math.sqrt(u**2 + w**2)
    V_Matrix = np.array([u, w])
    Mach_Number = V / a
    alpha = math.atan2(w, u)   # [rad] Angle of Attack 
    gamma = theta - alpha      # [rad] Flight Path Angle 
    Q_dyn = 0.5 * rho * V**2   # [Pa] Dynamic Pressure 
    
    # Coordinate Transformations
    # L_BE: Body to Earth (Rotation Matrix)
    L_BE = np.array([
        [math.cos(theta), -math.sin(theta)], 
        [math.sin(theta),  math.cos(theta)]
    ]) 
    L_EB = L_BE.T # .T is the NumPy equivalent of MATLAB's ' (transpose)
    
    # L_BS: Stability to Body
    L_BS = np.array([
        [math.cos(alpha), -math.sin(alpha)],
        [math.sin(alpha),  math.cos(alpha)]
    ])

    # =========================================================================
    # 3. AERODYNAMICS
    # =========================================================================
    # --- Lift Coefficient (Linear Model + Lookup Base) ---
    # Note: SciPy interpolators return arrays. We use float()[0] to extract the scalar number
    CL_base = float(ac['Aero']['F_CL']([math.degrees(alpha), Mach_Number])[0])
    CL      = CL_base + (CL_delta * delta_e)
    
    # --- Drag Coefficient (Polar Model) ---
    CD0 = float(ac['Aero']['F_Cd0']([Mach_Number])[0])
    K   = float(ac['Aero']['F_K']([CL, Mach_Number])[0])
    CD_SpeedBrake = 0.025 * d_sb
    CD_Gear       = 0.020 * d_gear
    CD  = CD0 + K * CL**2 + CD_SpeedBrake + CD_Gear
    
    # --- Moment Coefficient ---
    Cm  = Cm0 + (Cma * alpha) + (Cmd * delta_e)
    
    # --- Dimensional Forces & Moments ---
    L_force = Q_dyn * S_ref * CL  # Renamed from 'L' to avoid confusion with Lapse Rate or limits
    D_force = Q_dyn * S_ref * CD
    M_Aero  = Q_dyn * S_ref * c_bar * Cm
    
    # --- Stall Speed Calculation (Dynamic) ---
    Query_Mach = max(Mach_Number, min(ac['Aero']['Limits']['Mach_Vec'])) 
    # np.interp is the 1-to-1 equivalent of MATLAB's interp1 for 1D linear interpolation
    Current_CL_max = np.interp(
        Query_Mach, 
        ac['Aero']['Limits']['Mach_Vec'], 
        ac['Aero']['Limits']['CL_Max_Vec']
    )
    V_stall = math.sqrt(2 * m * g / (rho * S_ref * Current_CL_max))

    # =========================================================================
    # 4. PROPULSION (J85-GE-5 Engine Model)
    # =========================================================================
    Alt_Input = max(0.0, h)
    
    # A. Retrieve MAX Thrust Values [kN] from Maps
    T_max_dry_kN = float(ac['Engine']['Dry']['F_Thrust']([Alt_Input, Mach_Number])[0])
    T_max_wet_kN = float(ac['Engine']['Wet']['F_Thrust']([Alt_Input, Mach_Number])[0])
    
    SFC_dry_val  = float(ac['Engine']['Dry']['F_SFC']([Alt_Input, Mach_Number])[0])
    SFC_wet_val  = float(ac['Engine']['Wet']['F_SFC']([Alt_Input, Mach_Number])[0])
    
    # B. Throttle Logic
    AB_Threshold = 0.90
    
    if throttle_setting <= AB_Threshold:
        # --- DRY MODE ---
        pct_power = throttle_setting / AB_Threshold 
        T_single_kN = T_max_dry_kN * pct_power
        SFC_current = SFC_dry_val 
        AB_Status   = 0
    else:
        # --- WET MODE (AFTERBURNER) ---
        pct_ab = (throttle_setting - AB_Threshold) / (1.0 - AB_Threshold)
        T_single_kN = T_max_dry_kN + (T_max_wet_kN - T_max_dry_kN) * pct_ab
        SFC_current = SFC_dry_val + (SFC_wet_val - SFC_dry_val) * pct_ab
        AB_Status   = 1
    
    # C. Final Propulsion Outputs
    T_Total = (T_single_kN * 1000.0) * ac['Engine']['Count'] 
    m_dot_fuel = -(T_Total * SFC_current)

    # =========================================================================
    # 5. EQUATIONS OF MOTION
    # =========================================================================
    W = m * g
    F_stability = np.array([-D_force, -L_force])
    T_Matrix    = np.array([T_Total * math.cos(epsilon), -T_Total * math.sin(epsilon)])
    m_Matrix    = np.array([-m * g * math.sin(theta), m * g * math.cos(theta)])
    
    # Body-frame aerodynamic + thrust forces
    F_aerop_body = (L_BS @ F_stability) + T_Matrix
    
    # Standard Free-Flight Body Accelerations (u_dot, w_dot)
    u_dot_free = (F_aerop_body[0] + m_Matrix[0]) / m - q * w
    w_dot_free = (F_aerop_body[1] + m_Matrix[1]) / m + q * u

    V_E_Matrix = L_EB @ V_Matrix

    # --- GROUND BOUNDARY CHECK ---
    # Calculate static Normal Force required to balance vertical forces in Earth frame
    F_z_earth = -F_aerop_body[0]*math.sin(theta) + F_aerop_body[1]*math.cos(theta) + W
    N_force = max(0.0, F_z_earth)

    # Condition: We are physically at/below ground AND pushing down (N_force > 0)
    if z_E >= 0.0 and N_force > 0:
        
        # 1. Calculate Friction based on true Normal Force
        F_Friction = N_force * mu
        
        # 2. Earth frame horizontal acceleration (Thrust - Drag - Friction)
        F_x_earth = F_aerop_body[0]*math.cos(theta) + F_aerop_body[1]*math.sin(theta)
        V_dot_ground = (F_x_earth - F_Friction) / m
        
        # 3. KINEMATIC CONSTRAINT: Force body derivatives to keep flight path perfectly flat
        V_ground = V_E_Matrix[0] # Horizontal Earth speed
        u_dot = V_dot_ground * math.cos(theta) - V_ground * math.sin(theta) * q
        w_dot = V_dot_ground * math.sin(theta) + V_ground * math.cos(theta) * q
        
        # 4. Lock Earth-frame vertical velocity to 0
        x_dot_E = V_ground
        z_dot_E = 0.0 
        
    else:
        # --- FREE FLIGHT ---
        N_force = 0.0
        F_Friction = 0.0
        
        u_dot = u_dot_free
        w_dot = w_dot_free
        
        x_dot_E = V_E_Matrix[0]
        z_dot_E = V_E_Matrix[1]

    # Pitching Moment (Same for both phases for now)
    q_dot = M_Aero / I_yy
    theta_dot = q
    
    # Calculate Earth-Frame Accelerations strictly for logging/dashboard plotting
    # Applying the product rule: a_E = L_EB * a_B + L_EB_dot * V_B
    u_dot_E = u_dot * math.cos(theta) - u * q * math.sin(theta) + w_dot * math.sin(theta) + w * q * math.cos(theta)
    w_dot_E = -u_dot * math.sin(theta) - u * q * math.cos(theta) + w_dot * math.cos(theta) - w * q * math.sin(theta)
        
    # =========================================================================
    # 6. OUTPUT PACKAGING
    # =========================================================================
    output = {
        'Derivatives': {
            'x_dot_E': x_dot_E,   # Fixed: Using the explicitly defined variable
            'z_dot_E': z_dot_E,   # Fixed
            'u_dot': u_dot,       # Fixed: No longer calling a_Matrix
            'w_dot': w_dot,       # Fixed
            'theta_dot': theta_dot,
            'q_dot': q_dot,
            'm_dot': m_dot_fuel
        },
        'States': {
            'Alpha': alpha,
            'Gamma': gamma,
            'Delta_e': delta_e,
            'Throttle_Setting': throttle_setting,
            'AB_Status': AB_Status,
            'SFC': SFC_current,
            'V': V,
            'Mach_Number': Mach_Number,
            'Vstall': V_stall,
            'u_E': x_dot_E,       # Cleaned up reference
            'w_E': z_dot_E,       # Cleaned up reference
            'u_dot_E': u_dot_E,   # Fixed: No longer calling a_Matrix_E
            'w_dot_E': w_dot_E    # Fixed
        },
        'Forces': {
            'W': W,
            'L': L_force,
            'D': D_force,
            'T': T_Total,
            'N': N_force,
            'F_Friction': F_Friction
        },
        'Moments': {
            'M_Aero': M_Aero
        },
        'Controls': controls
    }
    
    return output