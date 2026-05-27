import pygame
import math
import pickle
import sys
import numpy as np
import pyvista as pv
from control_systems import run_pid
from pydualsense import pydualsense

# Import your existing modules
from T_38_General_Data import get_t38_general_data
from atmosphere import atmosisa
from flight_dynamics import flight_dynamics

def advance_state(state_dict, derivatives_dict, step_size):
    """Helper function to advance states by a given step size (used inside RK4)"""
    new_state = state_dict.copy()
    new_state['x_E']   += derivatives_dict['x_dot_E'] * step_size
    new_state['z_E']   += derivatives_dict['z_dot_E'] * step_size
    new_state['u']     += derivatives_dict['u_dot'] * step_size
    new_state['w']     += derivatives_dict['w_dot'] * step_size
    new_state['theta'] += derivatives_dict['theta_dot'] * step_size
    new_state['theta'] = (new_state['theta'] + math.pi) % (2.0 * math.pi) - math.pi
    new_state['q']     += derivatives_dict['q_dot'] * step_size
    new_state['m']     += derivatives_dict['m_dot'] * step_size
    return new_state

def rk4_step(State, AC, Env, Controls, dt):
    """Calculates the next state using 4th-Order Runge-Kutta"""
    out1 = flight_dynamics(State, AC, Env, Controls)
    k1 = out1['Derivatives']
    
    state2 = advance_state(State, k1, dt/2)
    out2 = flight_dynamics(state2, AC, Env, Controls)
    k2 = out2['Derivatives']
    
    state3 = advance_state(State, k2, dt/2)
    out3 = flight_dynamics(state3, AC, Env, Controls)
    k3 = out3['Derivatives']
    
    state4 = advance_state(State, k3, dt)
    out4 = flight_dynamics(state4, AC, Env, Controls)
    k4 = out4['Derivatives']
    
    Next_State = State.copy()
    Next_State['x_E']   += (dt/6) * (k1['x_dot_E'] + 2*k2['x_dot_E'] + 2*k3['x_dot_E'] + k4['x_dot_E'])
    Next_State['z_E']   += (dt/6) * (k1['z_dot_E'] + 2*k2['z_dot_E'] + 2*k3['z_dot_E'] + k4['z_dot_E'])
    Next_State['u']     += (dt/6) * (k1['u_dot']   + 2*k2['u_dot']   + 2*k3['u_dot']   + k4['u_dot'])
    Next_State['w']     += (dt/6) * (k1['w_dot']   + 2*k2['w_dot']   + 2*k3['w_dot']   + k4['w_dot'])
    Next_State['theta'] += (dt/6) * (k1['theta_dot']+ 2*k2['theta_dot']+ 2*k3['theta_dot']+ k4['theta_dot'])
    Next_State['q']     += (dt/6) * (k1['q_dot']   + 2*k2['q_dot']   + 2*k3['q_dot']   + k4['q_dot'])
    Next_State['m']     += (dt/6) * (k1['m_dot']   + 2*k2['m_dot']   + 2*k3['m_dot']   + k4['m_dot'])

    Next_State['theta'] = (Next_State['theta'] + math.pi) % (2.0 * math.pi) - math.pi
    
    return Next_State, out1 

# =========================================================================
# 0. INITIALIZE AUTOPILOT
# =========================================================================
print(">> Loading Autopilot Control Matrix...")
try:
    with open('t38_control_matrix.pkl', 'rb') as f:
        Autopilot_Matrix = pickle.load(f)
    print(">> Autopilot Matrix Online.")
except FileNotFoundError:
    print(">> ERROR: t38_control_matrix.pkl not found. AP offline.")
    Autopilot_Matrix = None

ap_engaged = False
target_alt = 0.0
target_mach = 0.0

# --- CASCADED PID CONFIGURATIONS (From MATLAB Optimization) ---
# 1. Outer Loop: Altitude controls Target Pitch (Clamped to +/- 5 deg)
pid_config_alt = {'Kp': 0.00157, 'Ki': 0.00011, 'Kd': -0.00210, 'Min': math.radians(-5), 'Max': math.radians(5)}

# 2. Inner Loop: Pitch error controls Elevator (Clamped to +/- 15 deg)
pid_config_pitch = {'Kp': -35.60, 'Ki': 0.0244, 'Kd': 24.31, 'Min': math.radians(-15), 'Max': math.radians(15)}

# 3. Speed Loop: Mach error controls Throttle (0% to 100%)
pid_config_thrust = {'Kp': 649.27, 'Ki': -0.0015, 'Kd': 2.88, 'Min': 0.0, 'Max': 1.0}

pid_state_alt = {'Integrator': 0.0, 'PrevError': 0.0}
pid_state_pitch = {'Integrator': 0.0, 'PrevError': 0.0}
pid_state_thrust = {'Integrator': 0.0, 'PrevError': 0.0}

# For gain scheduling
Q_base = 0.5 * 1.225 * (0.8 * 340.29)**2 # Tuned for Mach 0.8

# =========================================================================
# 1. INITIALIZATION: PHYSICS & PYGAME (INSTRUMENT PANEL)
# =========================================================================

# --- ORBITAL CAMERA SETUP ---
cam_azimuth = 0.0    # 0 degrees means looking straight from behind
cam_elevation = 15.0 # Looking slightly down at the aircraft
cam_distance = 100.0  # Distance in meters from the jet

pygame.init()

# --- JOYSTICK INITIALIZATION ---
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
for js in joysticks:
    js.init()

if joysticks:
    print(f">> Hardware Controller Detected: {joysticks[0].get_name()}")
else:
    print(">> No controller detected. Defaulting to keyboard.")

WIDTH, HEIGHT = 400, 500
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("T-38 Talon - Instruments (CLICK HERE TO FLY)")
font = pygame.font.SysFont("courier", 16)
clock = pygame.time.Clock()

dt = 0.01
AC = get_t38_general_data()

State = {'m': AC['Mass']['Initial'], 'x_E': 0.0, 'z_E': 0.0, 'u': 0.5, 'w': 0.0, 'theta': 0.0, 'q': 0.0}
Controls = {'ElevatorDeflection': 0.0, 'ThrottleSetting': 0.0, 'Gear': 1.0, 'SpeedBrake': 0.0}
# Initialize Log_Data to prevent reference errors before the first RK4 step
Log_Data = {'States': {'Mach_Number': 0.0}} 

# =========================================================================
# 2. INITIALIZATION: PYVISTA (3D OUT-THE-WINDOW VIEW)
# =========================================================================
print('>> [VIS] Initializing GPU-Accelerated Scene...')

try:
    t38_mesh = pv.read('T38.stl')
    t38_mesh.scale([0.1, 0.1, 0.1], inplace=True)
    t38_mesh.rotate_z(180, inplace=True) 
except Exception:
    t38_mesh = pv.Cone(direction=(-1, 0, 0), height=10, radius=3)

plotter = pv.Plotter(title="T-38 Live 3D View")
plotter.set_background('lightskyblue')

# Environment
earth = pv.Plane(center=[2000, 0, -0.2], direction=(0, 0, 1), 
                 i_size=20000, j_size=20000, 
                 i_resolution=400, j_resolution=400)
earth_actor = plotter.add_mesh(earth, color='forestgreen', show_edges=True, edge_color='#225522', ambient=0.3)
runway = pv.Cube(center=[950, 0, 0], bounds=[-100, 2000, -22.5, 22.5, -0.1, 0])
plotter.add_mesh(runway, color='#333333', ambient=0.3)

for x_dash in range(0, 2000, 60): 
    dash = pv.Cube(center=[x_dash + 15, 0, 0.05], bounds=[x_dash, x_dash + 30, -0.5, 0.5, 0.0, 0.1])
    plotter.add_mesh(dash, color='white', ambient=0.8)

# Actors
aircraft_actor = plotter.add_mesh(t38_mesh, color='#D9D9E6', specular=0.5, smooth_shading=True)

base_flame = pv.Cone(direction=(-1, 0, 0), height=4.0, radius=0.32)
flame_L_mesh = base_flame.translate([-20.5, -0.5, 1.2], inplace=False)
flame_R_mesh = base_flame.translate([-20.5,  0.5, 1.2], inplace=False)
flame_L_actor = plotter.add_mesh(flame_L_mesh, color='darkorange', ambient=1.0, opacity=0.8)
flame_R_actor = plotter.add_mesh(flame_R_mesh, color='darkorange', ambient=1.0, opacity=0.8)

plotter.show(interactive_update=True)

# =========================================================================
# 3. THE UNIFIED REAL-TIME LOOP
# =========================================================================

# --- INITIALIZE DUALSENSE HID ---
ds = pydualsense()
ds_connected = False
try:
    ds.init()
    ds_connected = True
    print(">> pydualsense connection established for HID haptics.")
except Exception as e:
    print(f">> pydualsense HID failed: {e}. Haptics disabled.")

prev_alpha = 0.0
running = True

while running:
    # --- A. PYGAME EVENT HANDLING ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
        # Keyboard Toggles
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_g:
                Controls['Gear'] = 0.0 if Controls['Gear'] == 1.0 else 1.0
            elif event.key == pygame.K_b:
                Controls['SpeedBrake'] = 0.0 if Controls['SpeedBrake'] == 1.0 else 1.0
            elif event.key == pygame.K_a and Autopilot_Matrix is not None:
                ap_engaged = not ap_engaged
                if ap_engaged:
                    target_alt = -State['z_E']
                    target_mach = Log_Data['States']['Mach_Number']
                    pid_state_pitch = {'Integrator': 0.0, 'PrevError': 0.0}
                    pid_state_thrust = {'Integrator': 0.0, 'PrevError': 0.0}
                    print(f">> AP ENGAGED: Holding {target_alt:.0f}m at Mach {target_mach:.2f}")
                else:
                    print(">> AP DISENGAGED: Manual Control.")
                    
        # Gamepad Toggles
        elif event.type == pygame.JOYBUTTONDOWN:
            if event.button == 3: # Triangle
                Controls['Gear'] = 0.0 if Controls['Gear'] == 1.0 else 1.0
            elif event.button == 1: # Circle
                Controls['SpeedBrake'] = 0.0 if Controls['SpeedBrake'] == 1.0 else 1.0
            elif event.button == 0 and Autopilot_Matrix is not None: # Cross Button
                ap_engaged = not ap_engaged
                if ap_engaged:
                    # Lock current states as targets
                    target_alt = -State['z_E']
                    target_mach = Log_Data['States']['Mach_Number']
                    # Reset all 3 Integrators
                    pid_state_alt = {'Integrator': 0.0, 'PrevError': 0.0}
                    pid_state_pitch = {'Integrator': 0.0, 'PrevError': 0.0}
                    pid_state_thrust = {'Integrator': 0.0, 'PrevError': 0.0}
                    print(f">> AP ENGAGED: Holding {target_alt:.0f}m at Mach {target_mach:.2f}")
                else:
                    print(">> AP DISENGAGED: Manual Control.")

    # --- B. FLIGHT CONTROLS (Autopilot vs Manual) ---
    keys = pygame.key.get_pressed()
    throttle_rate = 0.2 * dt 

    # ---> CAMERA ORBIT LOGIC <---
    if joysticks:
        js = joysticks[0]
        # Axis 2 = Right Stick X (Azimuth / Orbit Left/Right)
        # Axis 3 = Right Stick Y (Elevation / Orbit Up/Down)
        cam_x_axis = js.get_axis(2)
        cam_y_axis = js.get_axis(3)

        if abs(cam_x_axis) > 0.05:
            cam_azimuth += cam_x_axis * 2.0  # Rotate 2 degrees per frame
        if abs(cam_y_axis) > 0.05:
            cam_elevation += cam_y_axis * 2.0 
        
        # Clamp the elevation so you don't go under the ground or flip upside down
        cam_elevation = max(-5.0, min(80.0, cam_elevation))

    # ---> AIRCRAFT CONTROL LOGIC <---
    if ap_engaged:
        current_alt = -State['z_E']
        current_mach = max(0.2, Log_Data['States']['Mach_Number']) 
        curr_pitch = State['theta']
        
        # 1. FEED-FORWARD (The Matrix)
        base_thr = float(Autopilot_Matrix['Throttle_Map'](current_alt, current_mach))
        base_ele = float(Autopilot_Matrix['Elevator_Map'](current_alt, current_mach))
        
        # Handle edge cases where interpolation returns NaN (outside envelope)
        if math.isnan(base_thr) or math.isnan(base_ele):
            print(f">> AP REFUSED: Envelope violation at {current_alt:.0f}m, Mach {current_mach:.2f}. Disengaging.")
            ap_engaged = False
        else:
            # 2. GAIN SCHEDULING
            _, a_val, _, rho_val = atmosisa(current_alt)
            V_current = current_mach * a_val
            Q_current = max(1.0, 0.5 * rho_val * V_current**2) 
            q_ratio = Q_base / Q_current
            
            sched_pitch = pid_config_pitch.copy()
            sched_pitch['Kp'] *= q_ratio
            sched_pitch['Kd'] *= q_ratio
            
            # --- CASCADED FEEDBACK LOOP ---
            target_pitch_cmd, pid_state_alt = run_pid(target_alt, current_alt, dt, pid_config_alt, pid_state_alt)
            
            sched_pitch['Base'] = base_ele 
            cmd_ele, pid_state_pitch = run_pid(target_pitch_cmd, curr_pitch, dt, sched_pitch, pid_state_pitch, State['q'])
            Controls['ElevatorDeflection'] = cmd_ele
            
            pid_config_thrust['Base'] = base_thr
            cmd_thr, pid_state_thrust = run_pid(target_mach, current_mach, dt, pid_config_thrust, pid_state_thrust)
            Controls['ThrottleSetting'] = cmd_thr

    else:
        # MANUAL CONTROL
        if joysticks:
            js = joysticks[0]
            if js.get_button(10): Controls['ThrottleSetting'] += throttle_rate
            if js.get_button(9):  Controls['ThrottleSetting'] -= throttle_rate
            
            pitch_axis = js.get_axis(1)
            if abs(pitch_axis) > 0.05: 
                Controls['ElevatorDeflection'] = -pitch_axis * math.radians(15)
            else:
                Controls['ElevatorDeflection'] = 0.0
        else:
            if keys[pygame.K_LSHIFT]: Controls['ThrottleSetting'] += throttle_rate
            elif keys[pygame.K_LCTRL]: Controls['ThrottleSetting'] -= throttle_rate

            elev_rate = math.radians(30) * dt
            if keys[pygame.K_DOWN] or keys[pygame.K_s]: Controls['ElevatorDeflection'] -= elev_rate 
            elif keys[pygame.K_UP] or keys[pygame.K_w]: Controls['ElevatorDeflection'] += elev_rate 
            else:
                if Controls['ElevatorDeflection'] > 0.001: Controls['ElevatorDeflection'] -= elev_rate
                elif Controls['ElevatorDeflection'] < -0.001: Controls['ElevatorDeflection'] += elev_rate
                else: Controls['ElevatorDeflection'] = 0.0

    # ---> ENVELOPE PROTECTION (ALPHA & G-LIMITER) <---
    current_alpha = Log_Data['States'].get('Alpha', 0.0)
    current_q = State['q'] 

    forces = Log_Data.get('Forces', {'L': 0.0, 'N': State['m'] * 9.81})
    g_load = (forces['L'] + forces['N']) / (State['m'] * 9.81)
    
    # 1. Alpha Limiter Calculations
    alpha_limit_deg = 20.0
    alpha_buffer_deg = 2.0  
    alpha_threshold = math.radians(alpha_limit_deg - alpha_buffer_deg)
    
    alpha_override = 0.0
    if current_alpha > alpha_threshold:
        violation_a = current_alpha - alpha_threshold
        k_alpha_p = 15.0  
        k_alpha_d = 2.5   
        alpha_override = (k_alpha_p * violation_a) + (k_alpha_d * max(0.0, current_q))

    # 2. G-Limiter Calculations
    g_limit = 7.33
    g_buffer = 0.5 
    g_threshold = g_limit - g_buffer
    
    g_override = 0.0
    if g_load > g_threshold:
        violation_g = g_load - g_threshold
        # K_g_p: Radians of nose-down elevator per G over the limit
        k_g_p = 0.15 
        # K_g_d: Pitch rate damping to prevent snapping the nose down too violently
        k_g_d = 1.5  
        g_override = (k_g_p * violation_g) + (k_g_d * max(0.0, current_q))

    # 3. Max-Select Logic & Application
    # The FCC applies whichever limit requires the most aggressive nose-down correction
    active_override = max(alpha_override, g_override)
    
    limiter_status = "STANDBY"
    if active_override > 0.0:
        Controls['ElevatorDeflection'] += active_override
        if alpha_override > g_override:
            limiter_status = "ALPHA LIMIT (PITCH DOWN)"
        else:
            limiter_status = "G-LIMIT (PITCH DOWN)"

    # Clamp Final Control Limits
    Controls['ThrottleSetting'] = max(0.0, min(1.0, Controls['ThrottleSetting']))
    Controls['ElevatorDeflection'] = max(math.radians(-15), min(math.radians(15), Controls['ElevatorDeflection']))

    # --- C. RK4 PHYSICS INTEGRATION ---
    _, a_val, _, rho_val = atmosisa(-State['z_E'])
    Env = {'rho': rho_val, 'a': a_val, 'g': 9.81, 'mu': 0.04}
    
    Next_State, Log_Data = rk4_step(State, AC, Env, Controls, dt)
    State = Next_State

    # ---> TRANSONIC / SUPERSONIC RUMBLE (HID BYPASS) <---
    if ds_connected:
        mach_number = Log_Data['States']['Mach_Number']
        
        if mach_number >= 1.0:
            # Map Mach 1.0 - 1.5 to an integer range of 0 to 255
            intensity_float = 0.01 + 0.05 * min(1.0, (mach_number - 1.0) / 0.5)
            rumble_int = int(intensity_float * 255)
            
            ds.setLeftMotor(rumble_int)
            ds.setRightMotor(rumble_int)
        else:
            ds.setLeftMotor(0)
            ds.setRightMotor(0)

    # --- CRASH DETECTION LOGIC ---
    if State['z_E'] > 0.5:
        print(f"\n>> FATAL ERROR: Ground Impact Detected.")
        print(f"   Final Pitch : {math.degrees(State['theta']):.1f} deg")
        print(f"   Final Speed : {Log_Data['States']['V'] * 1.94384:.1f} kts")
        print(">> Simulation Terminated.")
        if ds_connected:
            ds.close()
        pygame.quit()
        plotter.close()
        sys.exit(1)

    # --- D. PYVISTA 3D RENDERING UPDATE ---
    curr_x = State['x_E']
    curr_z = -State['z_E'] 
    curr_pitch = math.degrees(State['theta'])

    matrix = np.eye(4)
    cos_p, sin_p = np.cos(np.radians(-curr_pitch)), np.sin(np.radians(-curr_pitch))
    matrix[0, 0], matrix[0, 2] = cos_p, sin_p
    matrix[2, 0], matrix[2, 2] = -sin_p, cos_p
    matrix[0, 3], matrix[1, 3], matrix[2, 3] = curr_x, 0.0, curr_z + 2.5

    aircraft_actor.user_matrix = matrix
    flame_L_actor.user_matrix = matrix
    flame_R_actor.user_matrix = matrix

    # ---> THE INFINITE EARTH TREADMILL <---
    # Shifts the earth mesh to stay exactly under the camera, 
    # snapping to the 50m resolution so the grid appears stationary.
    earth_matrix = np.eye(4)
    earth_matrix[0, 3] = (curr_x // 50) * 50 
    earth_actor.user_matrix = earth_matrix

    # ---> AFTERBURNER TRIGGER FIX (0.9 instead of 0.99) <---
    if Controls['ThrottleSetting'] >= 0.90:
        flame_L_actor.SetVisibility(True)
        flame_R_actor.SetVisibility(True)
        flicker = np.random.uniform(0.6, 1.0)
        flame_L_actor.prop.opacity = flicker
        flame_R_actor.prop.opacity = flicker
    else:
        flame_L_actor.SetVisibility(False)
        flame_R_actor.SetVisibility(False)

    # ---> DYNAMIC ORBITAL CAMERA <---
    # Convert angles to radians for the math functions
    az_rad = math.radians(cam_azimuth)
    el_rad = math.radians(cam_elevation)

    # Calculate offset using spherical coordinates
    offset_x = -cam_distance * math.cos(el_rad) * math.cos(az_rad)
    offset_y =  cam_distance * math.cos(el_rad) * math.sin(az_rad)
    offset_z =  cam_distance * math.sin(el_rad)

    # Apply the offsets to the aircraft's current position
    plotter.camera.position = [curr_x + offset_x, offset_y, curr_z + offset_z] 
    plotter.camera.focal_point = [curr_x, 0, curr_z]     
    
    plotter.update()

    # --- E. PYGAME HUD RENDERING (MATLAB PARITY) ---
    screen.fill((20, 20, 30))
    
    # 1. Base Kinematics
    alt_m = curr_z
    kias = Log_Data['States']['V'] * 1.94384
    mach = Log_Data['States']['Mach_Number']
    n_force = Log_Data['Forces']['N']
    aoa_deg = math.degrees(Log_Data['States']['Alpha'])
    gamma_deg = math.degrees(Log_Data['States']['Gamma'])
    roc = -Log_Data['States']['w_E'] 
    
    # 2. Advanced Telemetry (From your MATLAB code)
    T = Log_Data['Forces']['T']
    D = Log_Data['Forces']['D']
    W = State['m'] * 9.81
    V = Log_Data['States']['V']
   
    # Specific Excess Power (Converted from m/s to ft/min)
    sep_fpm = ((T - D) / W) * V * 196.85
    dist_km = curr_x / 1000.0
    
    # Fuel Flow and Mass
    m_dot = abs(Log_Data['Derivatives']['m_dot'])
    m_dot_safe = max(0.001, m_dot) # Singularity protection
    fuel_mass = State['m'] - 3500 + 250 # Empty mass + reserves based on your MATLAB script
    
    # Range and Endurance Logic
    if curr_z > 10:
        endur_min = (fuel_mass / m_dot_safe) / 60.0
        range_km = (V * (fuel_mass / m_dot_safe)) / 1000.0
        endur_str = f"{endur_min:8.1f} min"
        range_str = f"{range_km:8.0f} km"
    else:
        endur_str = "ON GROUND"
        range_str = "ON GROUND"
    
    hud_data = [
        f"--- FLIGHT COMPUTER ---",
        f"MACH NO  : {mach:.3f}",
        f"PITCH    : {curr_pitch:8.1f} deg",
        f"AOA      : {aoa_deg:8.1f} deg",        
        f"FLT PATH : {gamma_deg:8.1f} deg",    
        f"ALTITUDE : {alt_m:8.0f} m",
        f"G-LOAD   : {g_load:8.2f} G",
        f"THROTTLE : {Controls['ThrottleSetting']*100:8.1f} %",
        f"REHEAT   : {'ENGAGED' if Controls['ThrottleSetting'] >= 0.9 else 'OFF'}",
        f"SEP      : {sep_fpm:8.0f} ft/min",
        f"DIST FLWN: {dist_km:8.2f} km",
        f"FUEL FLOW: {m_dot:8.2f} kg/s",
        f"INST RANG: {range_str:>13}",
        f"INST ENDR: {endur_str:>13}",
        f"",
        f"--- CONFIGURATION ---",
        f"ELEVATOR : {math.degrees(Controls['ElevatorDeflection']):8.2f} deg",
        f"SPDBRAKE : {'EXTENDED' if Controls['SpeedBrake'] > 0.5 else 'RETRACTED'}",
        f"GEAR     : {'DOWN' if Controls['Gear'] > 0.5 else 'UP'}",
        f"W.O.W.   : {'YES' if n_force > 1.0 else 'NO'}",
        f"",
        f"AUTOPILOT: {'ENGAGED' if ap_engaged else 'OFF'}",
        f"ENV LIMIT: {limiter_status}"
    ]   

    for i, line in enumerate(hud_data):
        color = (0, 255, 0) 
        
        if "REHEAT" in line and "ENGAGED" in line:
            color = (255, 50, 50)
        elif "AUTOPILOT" in line and "ENGAGED" in line:
            color = (255, 0, 0)
        elif "ENV LIMIT" in line and "LIMIT" in line:  
            color = (255, 100, 0) # Flash Orange/Red
            
        screen.blit(font.render(line, True, color), (20, 20 + i * 20))
            
    pygame.display.flip()
    clock.tick(int(1/dt))
    # Store state for next derivative calculation
    prev_alpha = current_alpha

if ds_connected:
    ds.close()
pygame.quit()
plotter.close()
sys.exit()