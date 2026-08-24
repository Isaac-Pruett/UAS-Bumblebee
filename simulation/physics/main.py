import math
import time

import jsbsim
from pymavlink import mavutil

SIM_HZ = 200
DT = 1.0 / SIM_HZ

PX4_ADDR = "udpout:127.0.0.1:14560"


# ---------------------------------------------------------
# JSBSim
# ---------------------------------------------------------

fdm = jsbsim.FGFDMExec(None)

if not fdm.load_model("F450"):
    raise RuntimeError("Could not load F450 JSBSim model")

fdm.set_dt(DT)

# Initial conditions
fdm["ic/h-sl-ft"] = 10.0

fdm["ic/u-fps"] = 0.0
fdm["ic/v-fps"] = 0.0
fdm["ic/w-fps"] = 0.0

fdm["ic/phi-deg"] = 0.0
fdm["ic/theta-deg"] = 0.0
fdm["ic/psi-true-deg"] = 0.0

fdm.run_ic()


# ---------------------------------------------------------
# MAVLink connection to PX4 SITL
# ---------------------------------------------------------

mav = mavutil.mavlink_connection(
    PX4_ADDR,
    source_system=1,
    source_component=1,
)


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------

FT_TO_M = 0.3048
FPS_TO_MPS = 0.3048
FPS2_TO_MPS2 = 0.3048


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def euler_to_quaternion(roll, pitch, yaw):
    """
    Quaternion [w, x, y, z]
    """

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


# ---------------------------------------------------------
# Motor commands
# ---------------------------------------------------------

motor = [0.0, 0.0, 0.0, 0.0]


def process_px4_messages():
    global motor

    while True:
        msg = mav.recv_match(blocking=False)

        if msg is None:
            break

        if msg.get_type() == "HIL_ACTUATOR_CONTROLS":
            controls = msg.controls

            # For a quadrotor the first four actuator channels
            # are normally the four motors.
            #
            # Verify motor ordering against your PX4 airframe
            # and F450 JSBSim model.

            for i in range(4):
                motor[i] = clamp(controls[i], 0.0, 1.0)


# ---------------------------------------------------------
# PX4 sensor messages
# ---------------------------------------------------------


def send_hil_sensor(time_usec):

    # JSBSim body accelerations
    #
    # Check your F450's available properties.
    # These are common JSBSim properties.

    ax = fdm["accelerations/udot-ft_sec2"] * FPS2_TO_MPS2
    ay = fdm["accelerations/vdot-ft_sec2"] * FPS2_TO_MPS2
    az = fdm["accelerations/wdot-ft_sec2"] * FPS2_TO_MPS2

    p = fdm["velocities/p-rad_sec"]
    q = fdm["velocities/q-rad_sec"]
    r = fdm["velocities/r-rad_sec"]

    # Initially fake magnetometer.
    #
    # Replace later with magnetic-field model.
    xmag = 0.21523
    ymag = 0.0
    zmag = 0.42741

    # Initially fake atmosphere.
    abs_pressure = 1013.25
    diff_pressure = 0.0
    pressure_alt = fdm["position/h-sl-ft"] * FT_TO_M
    temperature = 20.0

    # Bitmask indicating which sensor fields changed.
    fields_updated = 0x1FFF

    mav.mav.hil_sensor_send(
        time_usec,
        ax,
        ay,
        az,
        p,
        q,
        r,
        xmag,
        ymag,
        zmag,
        abs_pressure,
        diff_pressure,
        pressure_alt,
        temperature,
        fields_updated,
    )


def send_hil_state(time_usec):

    roll = fdm["attitude/phi-rad"]
    pitch = fdm["attitude/theta-rad"]
    yaw = fdm["attitude/psi-rad"]

    q = euler_to_quaternion(
        roll,
        pitch,
        yaw,
    )

    p = fdm["velocities/p-rad_sec"]
    q_rate = fdm["velocities/q-rad_sec"]
    r = fdm["velocities/r-rad_sec"]

    # Body velocity
    u = fdm["velocities/u-fps"] * FPS_TO_MPS
    v = fdm["velocities/v-fps"] * FPS_TO_MPS
    w = fdm["velocities/w-fps"] * FPS_TO_MPS

    # Replace these with actual IC position if available.
    lat = 47.397742
    lon = 8.545594

    altitude_m = fdm["position/h-sl-ft"] * FT_TO_M

    # MAVLink HIL_STATE_QUATERNION:
    # lat/lon are degE7
    # altitude is mm

    mav.mav.hil_state_quaternion_send(
        time_usec,
        q,
        p,
        q_rate,
        r,
        int(lat * 1e7),
        int(lon * 1e7),
        int(altitude_m * 1000),
        int(u * 100),
        int(v * 100),
        int(w * 100),
        0,
        0,
        0,
    )


# ---------------------------------------------------------
# Apply PX4 motor outputs to JSBSim
# ---------------------------------------------------------


def apply_motors():

    # These MUST match the properties exposed by F450.xml.
    #
    # Rename as necessary.

    fdm["fcs/motor1-cmd-norm"] = motor[0]
    fdm["fcs/motor2-cmd-norm"] = motor[1]
    fdm["fcs/motor3-cmd-norm"] = motor[2]
    fdm["fcs/motor4-cmd-norm"] = motor[3]


# ---------------------------------------------------------
# Simulation
# ---------------------------------------------------------

sim_step = 0

while True:
    time_usec = int(fdm.get_sim_time() * 1_000_000)

    # PX4 -> simulator
    process_px4_messages()

    # Actuator outputs -> physics
    apply_motors()

    # Step JSBSim
    if not fdm.run():
        break

    # Simulator -> PX4
    #
    # IMU at 200 Hz
    send_hil_sensor(time_usec)

    # Ground-truth state at ~50 Hz
    if sim_step % 4 == 0:
        send_hil_state(time_usec)

    sim_step += 1

    # Initial/simple realtime pacing.
    #
    # Later we should replace this with proper synchronization
    # against PX4 simulator time.
    time.sleep(DT)
