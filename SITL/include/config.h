#pragma once

// ================================================================
// CONFIG.H  —  All tunable parameters in one place
// To add a drone: just increase DRONE_COUNT
// ================================================================

// ── Fleet ────────────────────────────────────────────────────────
#define DRONE_COUNT         3           // Number of drones to manage

// ── Network (SITL) ───────────────────────────────────────────────
#define UNITY_HOST          "127.0.0.1"
#define RECV_BASE_PORT      5000        // Unity sends state  to 5000+id
#define SEND_BASE_PORT      6000        // Unity expects cmds on 6000+id

// ── Timing ───────────────────────────────────────────────────────
#define CONTROL_LOOP_HZ     50          // Main control loop rate
#define STATE_TIMEOUT_MS    500         // Mark drone lost if no packet for this long

