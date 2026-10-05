# UAS-Bumblebee

---

Swarm flight stack

## Gamepad demo (PX4)

- **A:** arm the configured drones.
- **Left bumper:** request takeoff and initialize the formation center.
- **B:** start orbit-target rotation and request Offboard control for the configured
  drones (once per press). Wait for takeoff to finish before pressing B; this is
  an explicit handoff, not an automatic takeoff-completion check. Target rotation
  starts on the button press, even if a drone rejects Offboard.
- **Left stick:** move the shared center horizontally (each axis independently).
- **Right stick Y:** adjust orbit radius at up to 1 m/s; positive input increases
  it and negative input decreases it. Right-stick X is not required. Target
  altitude remains at launch height.

Orbit defaults are a **3 m radius** and **one counterclockwise revolution every
30 seconds**, viewed from above in ENU. Radius is limited to **1–20 m**, with a
0.1 stick deadzone. These constants are in
`gamepad_controlled_demo/src/orbit.rs`. Small radii may cause the existing
separation behavior to push drones away from their nominal targets.

The coordinator assigns evenly spaced target slots using the fixed `DRONES`
order and publishes each target as a MessagePack XYZ vector at approximately
20 Hz on `<drone>/formations/target`. Each formation node follows only its own
slot, retaining its existing cohesion, repulsion, and acceleration limits; the
actual flight path may lag the moving target. The shared `formations/target`
stream remains the blue reference sphere in telemetry, not a control target.
Restart both the coordinator and formation nodes together when upgrading from
the shared-target protocol.

The bridge requires an armed PX4 vehicle and a fresh heartbeat, primes Offboard
with at least 1.2 seconds of continuous outgoing setpoints, then requests the
mode and confirms it from a subsequent heartbeat. Keep `formations` running.
Rejected requests, missing streams, and timeouts are reported per drone; B does
not arm a vehicle or automatically retry a failed transition.
