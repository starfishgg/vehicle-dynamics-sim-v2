
# WHEEL IDENTIFIER ID's
FRONT_LEFT = 0
FRONT_RIGHT = 1
REAR_LEFT = 2
REAR_RIGHT = 3

# FUEL TYPE
PETROL = 0
DIESEL = 1

# FUEL WEIGHT
PETROL_WEIGHT = 0.74 # kg/l
DIESEL_WEIGHT = 0.875 # kg/l


# ============================================================
# ENGINE CONFIGURATIONS
# ============================================================

DIESEL_ENGINE = {
    "idle_rpm": 800,
    "peak_torque": 400,
    "base_torque": 150,
    "peak_rpm": 2250,
    "rise_width": 900,
    "fall_width": 1600,
    "redline": 5000,

    "idle_torque": 180,
    "engine_braking": 0.35,
    "friction_torque": 45, # Nm
    "inertia": 0.35,
}

NORMAL_PETROL_ENGINE = {
    "idle_rpm": 750,
    "peak_torque": 250,
    "base_torque": 100,
    "peak_rpm": 4000,
    "rise_width": 1600,
    "fall_width": 2400,
    "redline": 6500,

    "idle_torque": 110,
    "engine_braking": 0.50,
    "friction_torque": 25, # Nm
    "inertia": 0.18,
}

SPORTY_PETROL_ENGINE = {
    "idle_rpm": 850,
    "peak_torque": 400,
    "base_torque": 120,
    "peak_rpm": 4500,
    "rise_width": 1800,
    "fall_width": 3000,
    "redline": 7000,

    "idle_torque": 150,
    "engine_braking": 0.60,
    "friction_torque": 22, # Nm
    "inertia": 0.15,
}

HIGH_PERFORMANCE_NA_ENGINE = {
    "idle_rpm": 1000,
    "peak_torque": 500,
    "base_torque": 150,
    "peak_rpm": 6000,
    "rise_width": 2200,
    "fall_width": 3000,
    "redline": 8500,

    "idle_torque": 130,
    "engine_braking": 0.70,
    "friction_torque": 18, # Nm
    "inertia": 0.12,
}

HIGH_REV_SUPERCAR_ENGINE = {
    "idle_rpm": 1000,
    "peak_torque": 600,
    "base_torque": 180,
    "peak_rpm": 7000,
    "rise_width": 2600,
    "fall_width": 3200,
    "redline": 9500,

    "idle_torque": 140,
    "engine_braking": 0.75,
    "friction_torque": 16, # Nm
    "inertia": 0.10,
}


# ============================================================
# TYRE CONFIGURATIONS
# ============================================================

SPORTS_TYRE = {
    "B": 10.0,
    "C": 1.9,
    "D": 1.0,
    "E": 0.97,
}