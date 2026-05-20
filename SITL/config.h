#pragma once

// ================================================================
// CONFIG.H  —  All tunable parameters in one place
// To add a drone: just increase DRONE_COUNT
// ================================================================

// ── Fleet ────────────────────────────────────────────────────────
#define DRONE_COUNT         12           // Number of drones to manage

// ── Network (SITL) ───────────────────────────────────────────────
#define UNITY_HOST          "127.0.0.1"
#define RECV_BASE_PORT      5000        // Unity sends state  to 5000+id
#define SEND_BASE_PORT      6000        // Unity expects cmds on 6000+id

// ── Timing ───────────────────────────────────────────────────────
#define CONTROL_LOOP_HZ     50          // Main control loop rate
#define STATE_TIMEOUT_MS    3000        // Mark drone lost if no packet for this long

// ── Patrol grid ──────────────────────────────────────────────────
// Cell size is defined in GridPatrol.h: CELL_SIZE_M = 300m
// Arrival threshold: ARRIVAL_DIST = 80m

// ── Kalman filter noise (tune per hardware) ───────────────────────
// Q (process noise) and R (measurement noise) are in KalmanFilter.h
// Increase Q_lat/lon if GPS is trusted over IMU
// Decrease R_gps if your GPS is accurate

// ── Swarm LoRa ───────────────────────────────────────────────────
// Beacon size: 32 bytes (base64: 44 chars)
// Broadcast period: every BROADCAST_EVERY_N ticks (10 = 0.2s at 50Hz)
// Neighbour timeout: 5 seconds

// ── AIS avoidance ────────────────────────────────────────────────
// Alert range:  300m
// Danger range:  80m
// See TrajectoryController.h for full tuning

// ── Drone-drone avoidance ─────────────────────────────────────────
// Alert range:  150m
// Danger range:  40m
