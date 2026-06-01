# Airborne Flight Simulator — Project Brief for Claude Code

## What this project is

A concurrent backend service in Go that simulates a single aircraft and exposes a REST API for controlling it. Built as a senior developer home assignment. No external dependencies — standard library only.

## Current state (fully implemented and working)

### File structure
```
airborne/
├── main.go                    # Bootstrap, wiring, graceful shutdown, /map route
├── go.mod                     # module github.com/airborne, go 1.22
├── map.html                   # Browser live map (Leaflet + Chart.js, served at /map)
├── simulator/
│   ├── state.go               # All types: AircraftState, Command, Waypoint, FlightMode, EnvironmentSnapshot
│   ├── environment.go         # Environment interface + Wind, Humidity, Terrain, Multi implementations
│   └── simulator.go           # Actor goroutine, tick loop, step, moveToward, stepHold
└── api/
    └── handler.go             # All HTTP handlers + SSE stream
```

### Architecture
- **Actor model**: one goroutine owns all mutable state. No mutexes, no shared memory.
- **Channel-of-channels pattern** for state reads: API sends a `chan AircraftState` on `StateReqChan`, simulator writes the snapshot, API reads it back.
- **Command channel** (`CmdChan`, buffered 100): API sends `Command` structs, simulator processes them in its select loop.
- **20 Hz tick** (50ms) using `time.Ticker` with real wall-clock `dt` each step.
- **Clean shutdown**: `context.WithCancel` + OS signal → `srv.Shutdown` with 5s timeout.

### Flight modes (`FlightMode` in state.go)
| Mode | Meaning |
|------|---------|
| `idle` | No command, no thrust. Wind drifts position. |
| `flying` | Navigating to a waypoint or along a trajectory. |
| `holding` | Position-hold loop active — actively counters wind drift. |
| `stopped` | Explicit stop command. Zero thrust, free wind drift (same as idle but explicit). |

### Commands
| Endpoint | Behavior |
|----------|---------|
| `POST /command/goto` | Fly to single target `{lat, lon, alt, speed?}`. Auto-engages hold on arrival. |
| `POST /command/trajectory` | Fly list of waypoints `{waypoints:[...], loop:bool}`. Auto-hold at end if not looping. |
| `POST /command/hold` | Lock current position. Proportional controller fights wind. `vx/vy/vz` shows corrective thrust; `speed` shows actual net movement (≈0 when stable). |
| `POST /command/stop` | Zero thrust. Aircraft drifts freely. **Different from hold**: no position correction. |
| `GET /state` | Full state snapshot as JSON. |
| `GET /stream` | Server-Sent Events at 10 Hz. |
| `GET /health` | `{"status":"ok"}` |
| `GET /map` | Serves map.html |

### State response fields (all endpoints)
```json
{
  "lat": 32.0055,           // position
  "lon": 34.8854,
  "alt": 500.0,             // metres ASL
  "mode": "flying",         // idle | flying | holding | stopped
  "speed": 97.5,            // horizontal ground speed, own thrust only (m/s)
  "speed_3d": 98.1,         // full 3-D speed including vz (m/s)
  "heading": 45.2,          // degrees clockwise from north [0,360)
  "vx": 68.9,               // east component, own thrust (m/s)
  "vy": 69.2,               // north component, own thrust (m/s)
  "vz": 0.5,                // vertical component (m/s)
  "wind_vx": 5.0,           // eastward wind (m/s) — drifts position, not in vx/vy
  "wind_vy": 2.0,           // northward wind (m/s)
  "wind_vz": 0.0,           // vertical wind (m/s)
  "humidity_factor": 0.95,  // thrust multiplier [0,1]; 1.0 = no effect
  "terrain_floor": 50.0,    // minimum safe altitude (m ASL) = ground + 50m margin
  "timestamp": "2026-06-01T10:00:00Z"
}
```

### Key simulator constants (simulator.go)
| Constant | Value | Meaning |
|----------|-------|---------|
| `DefaultCruiseSpeed` | 100 m/s | Speed when none specified in command |
| `DefaultClimbRate` | 20 m/s | Max vertical speed |
| `ArrivalTolerance` | 20 m | 3-D distance at which waypoint is "reached" |
| `BrakingDistance` | 200 m | Distance at which deceleration begins |
| `MinApproachSpeed` | 5 m/s | Floor speed during braking |
| `HoldMaxThrust` | 30 m/s | Max corrective thrust in position-hold |
| `HoldProportionalGain` | 0.4 | P-gain: 1m error → 0.4 m/s correction |

### Environment design
Wind, Humidity, and Terrain each implement the `Environment` interface:
```go
type Environment interface {
    Apply(state AircraftState, dt time.Duration) AircraftState
    Snapshot() EnvironmentSnapshot
}
```
- **Wind**: drifts position only (`Lon`, `Lat`, `Alt`). Does NOT touch `Vel` or `Speed`. This keeps the velocity vector as "own thrust only" and makes the hold controller math clean.
- **Humidity**: multiplies `Vel` and `Speed` by `Factor` (0.95 = 5% performance loss). Only applies when `Speed > 0`.
- **Terrain**: clamps `Alt` to `GroundElevation + 50m`. Zeroes `Vel.VZ` if descending into terrain.
- **MultiEnvironment**: chains them in order. `Snapshot()` merges all child snapshots.

### Position-hold controller (stepHold in simulator.go)
Proportional (P-only) controller. Each tick:
1. Compute position error in metres from hold target.
2. Apply corrective thrust = `error × 0.4` (clamped to ±30 m/s).
3. Update position.
4. Then `env.Apply()` runs (wind drifts position back slightly).
5. `Vel` = corrective thrust vector (shows the drone is working).
6. `Speed` = actual net position change per second (≈0 when stable against wind).

**Known limitation**: pure P-controller has steady-state error. With 5 m/s wind and gain 0.4, equilibrium error ≈ 12.5m. A PI controller would eliminate this.

### Flat-earth coordinate math
```
metersPerDegreeLat = 111,320 m/°  (constant)
metersPerDegreeLon = cos(lat) × 111,320 m/°  (varies with latitude)
```
Used everywhere: moveToward, stepHold, WindEnvironment.Apply.

### map.html features
- Leaflet map with high-visibility yellow SVG drone icon (rotates with heading).
- Right-click on map → "Fly here" prompt → sends goto command directly.
- Trail toggle (ON/OFF).
- Re-center button + auto-center on new data, disengages on manual pan.
- Sidebar: position, velocity, environment panels.
- Three live charts: speed (horiz + 3D), altitude, velocity components (vx/vy/vz).
- Mode badge: color-coded (blue=flying, green=holding, red=stopped, grey=idle).

### Default startup config (main.go)
- Initial position: 32.0055°N, 34.8854°E (Ben Gurion Airport, Israel), 500m alt.
- Wind: 5 m/s east, 2 m/s north.
- Humidity factor: 0.95.
- Terrain: ground elevation 0m (sea level), floor = 50m ASL.

## Known issues / future improvements

1. **PI controller for hold**: current P-only controller has ~12.5m steady-state error against 5 m/s wind. Adding an integral term would drive error to zero.
2. **Wind is constant**: hardcoded at startup. A real system would have time-varying or spatially-varying wind.
3. **No acceleration model**: aircraft jumps to cruise speed instantly. Could add linear acceleration phase.
4. **Humidity compounds**: 0.95 multiplied per-tick is physically weird; should be applied once to the target speed, not to the velocity vector each tick. This causes slight speed reduction below requested value (189.99 instead of 200 for a 200 m/s command).
5. **Map doesn't auto-draw waypoints from external commands**: if you send trajectory from PowerShell the map won't show the waypoints (no feedback loop from server to map about active command).
6. **No /command/resume**: after hold, a new goto/trajectory is the only way to move again. Could add resume that re-activates the saved command.

## How to run

```bash
cd airborne
go run .
# Server on :8080
# Map at http://localhost:8080/map
```

## PowerShell test commands

```powershell
# Health
Invoke-RestMethod http://localhost:8080/health

# State
Invoke-RestMethod http://localhost:8080/state

# Goto with speed
Invoke-RestMethod -Method POST -Uri http://localhost:8080/command/goto `
  -ContentType "application/json" `
  -Body '{"lat":32.1,"lon":34.9,"alt":1000,"speed":150}'

# Trajectory with loop
Invoke-RestMethod -Method POST -Uri http://localhost:8080/command/trajectory `
  -ContentType "application/json" `
  -Body '{"waypoints":[{"lat":32.1,"lon":34.8,"alt":600},{"lat":32.2,"lon":35.0,"alt":800}],"loop":true}'

# Hold (position-lock, fights wind)
Invoke-RestMethod -Method POST -Uri http://localhost:8080/command/hold

# Stop (zero thrust, drifts with wind)
Invoke-RestMethod -Method POST -Uri http://localhost:8080/command/stop

# Watch state
while ($true) { Invoke-RestMethod http://localhost:8080/state | ConvertTo-Json; Start-Sleep -Milliseconds 500 }
```