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

// ── Controller gains (simple heading+speed PID for now) ──────────
#define KP_HEADING          0.15f
#define KD_HEADING          0.05f
#define KP_SPEED            0.5f

// ── Safety ───────────────────────────────────────────────────────
#define MAX_THROTTLE        1.0f
#define MAX_STEER           1.0f
#define MIN_AIS_DISTANCE    20.0f       // Metres — trigger avoidance below this
