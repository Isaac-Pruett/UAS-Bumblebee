# UAS-Bumblebee

---

Swarm flight stack

## Gamepad demo (PX4)

- **A:** arm the configured drones.
- **Left bumper:** request takeoff and initialize the formation center.
- **B:** request Offboard control for the configured drones (once per press).
  Wait for takeoff to finish before pressing B; this is an explicit handoff, not
  an automatic takeoff-completion check.

The bridge requires an armed PX4 vehicle and a fresh heartbeat, primes Offboard
with at least 1.2 seconds of continuous outgoing setpoints, then requests the
mode and confirms it from a subsequent heartbeat. Keep `formations` running.
Rejected requests, missing streams, and timeouts are reported per drone; B does
not arm a vehicle or automatically retry a failed transition.
