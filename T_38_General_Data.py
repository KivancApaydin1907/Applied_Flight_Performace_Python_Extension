import os
import numpy as np
import pickle
from scipy.interpolate import RegularGridInterpolator

def get_t38_general_data():
    """
    Aircraft Configuration Generator for Northrop T-38 Talon
    Returns a dictionary containing all physical, aerodynamic, and propulsion data.
    """
    # Initialize the main dictionary 
    airplane = {
        'Name': 'Northrop T-38 Talon'
    }
    
    # =========================================================================
    # 1. GEOMETRY & MASS PROPERTIES
    # =========================================================================
    airplane['Geo'] = {
        'S_ref': 15.8,  # Wing Reference Area [m^2]
        'c_bar': 2.36   # Mean Aerodynamic Chord [m]
    }
    airplane['Mass'] = {
        'Initial': 4536,  # Takeoff Mass [kg]
        'Iyy': 35080      # Pitch Moment of Inertia [kg*m^2]
    }
    
    # =========================================================================
    # PATH RESOLUTION
    # =========================================================================
    current_dir = os.path.dirname(os.path.abspath(__file__))
    aero_path = os.path.join(current_dir, 'T38_Aero_Data.pkl')
    engine_path = os.path.join(current_dir, 'J85_Engine_Performance.pkl')
    
    # =========================================================================
    # 2. AERODYNAMICS (LOOKUP TABLES)
    # =========================================================================
    if not os.path.isfile(aero_path):
        raise FileNotFoundError(f'Error: "T38_Aero_Data.pkl" not found at {aero_path}')
    
    # Load the pure Python dictionary instantly
    with open(aero_path, 'rb') as f:
        aero_db = pickle.load(f)
        
    airplane['Aero'] = {}
    
    # --- INTERPOLATORS ---
    airplane['Aero']['F_CL'] = RegularGridInterpolator(
        (aero_db['Lift']['Alpha_Vector'], aero_db['Lift']['Mach_Vector']), 
        aero_db['Lift']['CL_Matrix'], 
        method='linear', bounds_error = False, fill_value = None
    )
    
    airplane['Aero']['F_K'] = RegularGridInterpolator(
        (aero_db['Drag']['CL_Vector'], aero_db['Drag']['Mach_Vector']), 
        aero_db['Drag']['K_Matrix'], 
        method='linear', bounds_error = False, fill_value = None
    )
    
    # 1D interpolation for Cd0
    airplane['Aero']['F_Cd0'] = RegularGridInterpolator(
        (aero_db['Drag']['Cd0_MachVector'],), 
        aero_db['Drag']['Cd0_Vector'], 
        method='linear', bounds_error = False, fill_value = None
    )
    
    # Stability Derivatives
    airplane['Aero']['Cm0']      = aero_db['Moment']['Cm0']
    airplane['Aero']['Cm_alpha'] = aero_db['Moment']['Cm_alpha']
    airplane['Aero']['Cm_delta'] = aero_db['Moment']['Cm_delta']
    airplane['Aero']['CL_delta'] = aero_db['Lift']['CL_delta']
    
    # Operational Limits
    airplane['Aero']['Limits'] = {
        'CL_Max_Vec': np.max(aero_db['Lift']['CL_Matrix'], axis=0),
        'Mach_Vec':   aero_db['Lift']['Mach_Vector']
    }
    
    # =========================================================================
    # 3. PROPULSION SYSTEM (J85-GE-5)
    # =========================================================================
    if not os.path.isfile(engine_path):
        raise FileNotFoundError(f'Error: "J85_Engine_Performance.pkl" not found at {engine_path}')
    
    with open(engine_path, 'rb') as f:
        engine_db = pickle.load(f)
    
    # Extract the 1D grid axes from the 17x17 arrays
    h_vec = engine_db['Dry']['Altitude'][:, 0]  
    M_vec = engine_db['Dry']['Mach'][0, :]      
    
    SFC_SI_Factor = 1e-6 
    
    airplane['Engine'] = {
        'Count': 2,
        'epsilon': 0.0,
        'Dry': {},
        'Wet': {}
    }
    
    airplane['Engine']['Dry']['F_Thrust'] = RegularGridInterpolator(
        (h_vec, M_vec), engine_db['Dry']['Thrust'], method='linear', bounds_error = False, fill_value = None
    )
    airplane['Engine']['Dry']['F_SFC'] = RegularGridInterpolator(
        (h_vec, M_vec), engine_db['Dry']['SFC'] * SFC_SI_Factor, method='linear', bounds_error = False, fill_value = None
    )
    
    airplane['Engine']['Wet']['F_Thrust'] = RegularGridInterpolator(
        (h_vec, M_vec), engine_db['Wet']['Thrust'], method='linear', bounds_error = False, fill_value = None
    )
    airplane['Engine']['Wet']['F_SFC'] = RegularGridInterpolator(
        (h_vec, M_vec), engine_db['Wet']['SFC'] * SFC_SI_Factor, method='linear', bounds_error = False, fill_value = None
    )
    
    return airplane

# =========================================================================
# EXECUTION BLOCK
# =========================================================================
if __name__ == "__main__":
    print(">> Initializing T-38 General Data (Pure Python)...")
    AC = get_t38_general_data()
    print(f">> Success! Aircraft loaded: {AC['Name']}")
