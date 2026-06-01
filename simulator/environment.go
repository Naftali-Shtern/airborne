package simulator

import (
	"log"
	"math"
	"time"
)

// Environment applies external effects to the aircraft state each tick.
// Implementations must be pure – they receive a copy of state and return
// a modified copy; they must not store mutable references to the state.
type Environment interface {
	Apply(state AircraftState, dt time.Duration) AircraftState
	// Snapshot returns the current parameters for observability.
	Snapshot() EnvironmentSnapshot
}

// ---- Wind ---------------------------------------------------------------

// WindEnvironment drifts the aircraft *position* by a constant wind vector
// each tick. It does NOT modify Vel or Speed — those reflect only the
// aircraft's own thrust. Wind is a pure positional offset so that:
//   - Arriving at a waypoint always correctly zeroes Vel/Speed.
//   - Position-hold can compute the exact corrective thrust needed.
type WindEnvironment struct {
	WindVX float64 // eastward wind component  (m/s)
	WindVY float64 // northward wind component (m/s)
	WindVZ float64 // vertical wind component  (m/s)
}

func (w WindEnvironment) Apply(state AircraftState, dt time.Duration) AircraftState {
	dtSec := dt.Seconds()
	mPerLon := math.Cos(state.Lat*math.Pi/180.0) * 111_320.0
	state.Lon += (w.WindVX * dtSec) / mPerLon
	state.Lat += (w.WindVY * dtSec) / 111_320.0
	state.Alt += w.WindVZ * dtSec
	return state
}

func (w WindEnvironment) Snapshot() EnvironmentSnapshot {
	return EnvironmentSnapshot{WindVX: w.WindVX, WindVY: w.WindVY, WindVZ: w.WindVZ}
}

// ---- Humidity -----------------------------------------------------------

// HumidityEnvironment scales the aircraft's own velocity and speed by a
// factor in (0, 1]. Represents reduced engine performance in dense humid air.
// Only attenuates existing thrust; never adds velocity.
type HumidityEnvironment struct {
	Factor float64 // performance multiplier, clamped to (0, 1]
}

func (h HumidityEnvironment) Apply(state AircraftState, _ time.Duration) AircraftState {
	if state.Speed == 0 {
		return state // no thrust to attenuate
	}
	f := h.Factor
	if f <= 0 {
		f = 0.01
	}
	if f > 1 {
		f = 1
	}
	state.Vel.VX *= f
	state.Vel.VY *= f
	state.Vel.VZ *= f
	state.Speed *= f
	return state
}

func (h HumidityEnvironment) Snapshot() EnvironmentSnapshot {
	return EnvironmentSnapshot{HumidityFactor: h.Factor}
}

// ---- Terrain ------------------------------------------------------------

const terrainSafetyMarginMeters = 50.0

// TerrainEnvironment enforces a minimum altitude above a simple flat-terrain
// model. In a real system this would consult a DEM.
type TerrainEnvironment struct {
	GroundElevation float64 // metres ASL
}

func (t TerrainEnvironment) Apply(state AircraftState, _ time.Duration) AircraftState {
	floor := t.GroundElevation + terrainSafetyMarginMeters
	if state.Alt < floor {
		log.Printf("[terrain] alt %.1f m below floor %.1f m – clamping", state.Alt, floor)
		state.Alt = floor
		if state.Vel.VZ < 0 {
			state.Vel.VZ = 0
		}
	}
	return state
}

func (t TerrainEnvironment) Snapshot() EnvironmentSnapshot {
	return EnvironmentSnapshot{TerrainFloor: t.GroundElevation + terrainSafetyMarginMeters}
}

// ---- Composite ----------------------------------------------------------

// MultiEnvironment chains multiple Environment implementations in order.
type MultiEnvironment struct {
	Envs []Environment
}

func (m MultiEnvironment) Apply(state AircraftState, dt time.Duration) AircraftState {
	s := state
	for _, e := range m.Envs {
		s = e.Apply(s, dt)
	}
	return s
}

// Snapshot merges all child snapshots into one (last write wins per field).
func (m MultiEnvironment) Snapshot() EnvironmentSnapshot {
	var snap EnvironmentSnapshot
	for _, e := range m.Envs {
		s := e.Snapshot()
		if s.WindVX != 0 {
			snap.WindVX = s.WindVX
		}
		if s.WindVY != 0 {
			snap.WindVY = s.WindVY
		}
		if s.WindVZ != 0 {
			snap.WindVZ = s.WindVZ
		}
		if s.HumidityFactor != 0 {
			snap.HumidityFactor = s.HumidityFactor
		}
		if s.TerrainFloor != 0 {
			snap.TerrainFloor = s.TerrainFloor
		}
	}
	return snap
}
