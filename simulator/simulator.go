package simulator

import (
	"context"
	"log"
	"math"
	"time"
)

const (
	// DefaultTickRate is 20 Hz – within the required 10–60 Hz range.
	DefaultTickRate = 50 * time.Millisecond

	// DefaultCruiseSpeed is used when a waypoint carries no speed override (m/s).
	// ~360 km/h, typical for a fast UAV or small aircraft.
	DefaultCruiseSpeed = 100.0

	// DefaultClimbRate caps vertical speed so altitude changes are smooth (m/s).
	DefaultClimbRate = 20.0

	// ArrivalTolerance is the 3-D distance (m) at which a waypoint is
	// considered reached. Must exceed max distance per tick at MinApproachSpeed:
	//   5 m/s × 0.05 s = 0.25 m  →  20 m gives a comfortable margin.
	ArrivalTolerance = 20.0

	// BrakingDistance (m): aircraft starts decelerating this far from target.
	// At 100 m/s this gives a ~2 s ramp; at 200 m/s ~1 s ramp.
	// Kept short so the approach feels snappy (~15 s max at low speed).
	BrakingDistance = 200.0

	// MinApproachSpeed is the floor speed during braking (m/s).
	// Must be low enough that the aircraft reliably enters ArrivalTolerance.
	MinApproachSpeed = 5.0

	// HoldMaxThrust caps the corrective thrust used in position-hold (m/s).
	// Prevents aggressive oscillation when the wind is strong.
	HoldMaxThrust = 30.0

	// HoldProportionalGain maps position error (m) to corrective speed (m/s).
	// At 50 m error → 50 × 0.4 = 20 m/s correction.
	HoldProportionalGain = 0.4

	metersPerDegreeLat = 111_320.0
)

func metersPerDegreeLon(latDeg float64) float64 {
	return math.Cos(latDeg*math.Pi/180.0) * metersPerDegreeLat
}

// Simulator owns the mutable AircraftState and runs a fixed-rate tick loop.
// All external access is mediated through typed channels – no shared mutable
// state escapes the goroutine.
type Simulator struct {
	CmdChan      chan Command
	StateReqChan chan chan AircraftState

	state      AircraftState
	env        Environment
	tick       time.Duration
	lastUpdate time.Time

	// Hold state
	holdLat float64 // target lat for position-hold
	holdLon float64 // target lon for position-hold
	holdAlt float64 // target alt for position-hold
}

func New(initial AircraftState, env Environment, tick time.Duration) *Simulator {
	if tick <= 0 {
		tick = DefaultTickRate
	}
	// Embed initial env snapshot into state.
	if env != nil {
		initial.Env = env.Snapshot()
	}
	return &Simulator{
		CmdChan:      make(chan Command, 100),
		StateReqChan: make(chan chan AircraftState),
		state:        initial,
		env:          env,
		tick:         tick,
	}
}

func (s *Simulator) Run(ctx context.Context) {
	ticker := time.NewTicker(s.tick)
	defer ticker.Stop()
	s.lastUpdate = time.Now()

	var activeCmd *Command
	var waypointIndex int

	log.Printf("[sim] started at %.4f°N %.4f°E alt=%.1fm tick=%v cruiseSpeed=%.0fm/s",
		s.state.Lat, s.state.Lon, s.state.Alt, s.tick, DefaultCruiseSpeed)

	for {
		select {
		case <-ctx.Done():
			log.Println("[sim] shutting down")
			return

		case cmd := <-s.CmdChan:
			s.handleCommand(cmd, &activeCmd, &waypointIndex)

		case respChan := <-s.StateReqChan:
			respChan <- s.state

		case now := <-ticker.C:
			dt := now.Sub(s.lastUpdate)
			s.lastUpdate = now
			s.step(dt, &activeCmd, &waypointIndex)
		}
	}
}

func (s *Simulator) handleCommand(cmd Command, activeCmd **Command, waypointIdx *int) {
	switch cmd.Type {

	case CommandStop:
		// Full stop: zero thrust, no position correction. Aircraft drifts with wind.
		log.Println("[sim] STOP – zeroing thrust, aircraft will drift with wind")
		*activeCmd = nil
		s.state.Mode = ModeStopped
		s.state.Vel = Velocity{}
		s.state.Speed = 0

	case CommandHold:
		// Position-hold: lock onto current position and actively fight wind.
		log.Printf("[sim] HOLD – locking position at (%.4f, %.4f, %.1f)",
			s.state.Lat, s.state.Lon, s.state.Alt)
		s.engageHold()
		*activeCmd = nil // hold is driven by the hold loop, not activeCmd

	default:
		log.Printf("[sim] new command type=%s", cmd.Type)
		*activeCmd = &cmd
		*waypointIdx = 0
		s.state.Mode = ModeFlying
	}
	s.checkTerrainPath(cmd)
}

// checkTerrainPath logs a warning for every waypoint in cmd that would place
// the aircraft below the terrain safety floor. No-op for stop/hold or when the
// environment does not implement PathChecker.
func (s *Simulator) checkTerrainPath(cmd Command) {
	pc, ok := s.env.(PathChecker)
	if !ok {
		return
	}
	var waypoints []Waypoint
	switch cmd.Type {
	case CommandGoto:
		if cmd.Target != nil {
			waypoints = []Waypoint{*cmd.Target}
		}
	case CommandTrajectory:
		waypoints = cmd.Waypoints
	default:
		return
	}
	for _, w := range pc.CheckPath(s.state.Alt, waypoints) {
		log.Printf("[terrain-warning] %s", w)
	}
}

// engageHold freezes the current position as the hold target and switches mode.
func (s *Simulator) engageHold() {
	s.holdLat = s.state.Lat
	s.holdLon = s.state.Lon
	s.holdAlt = s.state.Alt
	s.state.Mode = ModeHolding
	s.state.Vel = Velocity{}
	s.state.Speed = 0
}

func (s *Simulator) step(dt time.Duration, activeCmd **Command, waypointIdx *int) {
	dtSec := dt.Seconds()

	switch s.state.Mode {

	case ModeHolding:
		// Position-hold loop: apply corrective thrust proportional to drift error,
		// then let wind drift the position, then update env snapshot.
		s.stepHold(dtSec)
		if s.env != nil {
			s.state = s.env.Apply(s.state, dt)
		}

	case ModeStopped, ModeIdle:
		// No thrust; aircraft drifts freely with wind.
		s.state.Vel = Velocity{}
		s.state.Speed = 0
		if s.env != nil {
			vel := s.state.Vel
			spd := s.state.Speed
			s.state = s.env.Apply(s.state, dt)
			s.state.Vel = vel // restore: env must not introduce thrust
			s.state.Speed = spd
		}

	case ModeFlying:
		if *activeCmd == nil {
			s.state.Mode = ModeIdle
			return
		}
		switch (*activeCmd).Type {
		case CommandGoto:
			done := s.moveToward((*activeCmd).Target, dtSec)
			if done {
				log.Printf("[sim] goto reached (%.4f, %.4f, %.1f) – engaging position-hold",
					(*activeCmd).Target.Lat, (*activeCmd).Target.Lon, (*activeCmd).Target.Alt)
				*activeCmd = nil
				s.engageHold() // auto-hold on arrival
			}

		case CommandTrajectory:
			wps := (*activeCmd).Waypoints
			if len(wps) == 0 {
				*activeCmd = nil
				s.state.Mode = ModeIdle
				return
			}
			done := s.moveToward(&wps[*waypointIdx], dtSec)
			if done {
				log.Printf("[sim] trajectory waypoint %d/%d reached", *waypointIdx+1, len(wps))
				*waypointIdx++
				if *waypointIdx >= len(wps) {
					if (*activeCmd).Loop {
						log.Println("[sim] trajectory loop – restarting")
						*waypointIdx = 0
					} else {
						log.Println("[sim] trajectory complete – engaging position-hold")
						*activeCmd = nil
						s.engageHold() // auto-hold at end of trajectory
					}
				}
			}
		}

		if s.env != nil {
			s.state = s.env.Apply(s.state, dt)
		}
	}

	// Always update env snapshot, active command snapshot, and timestamp.
	if s.env != nil {
		s.state.Env = s.env.Snapshot()
	}
	s.updateActiveSnapshot(*activeCmd, *waypointIdx)
	s.state.Timestamp = time.Now()
}

// updateActiveSnapshot writes the current command into state so the map
// can poll it without a separate channel.
func (s *Simulator) updateActiveSnapshot(activeCmd *Command, waypointIdx int) {
	if s.state.Mode == ModeHolding {
		holdWp := Waypoint{Lat: s.holdLat, Lon: s.holdLon, Alt: s.holdAlt}
		s.state.Active = ActiveCommandSnapshot{Type: "hold", Target: &holdWp}
		return
	}
	if activeCmd == nil {
		s.state.Active = ActiveCommandSnapshot{}
		return
	}
	switch activeCmd.Type {
	case CommandGoto:
		s.state.Active = ActiveCommandSnapshot{Type: "goto", Target: activeCmd.Target}
	case CommandTrajectory:
		s.state.Active = ActiveCommandSnapshot{
			Type: "trajectory", Waypoints: activeCmd.Waypoints,
			Loop: activeCmd.Loop, WaypointIndex: waypointIdx,
		}
	}
}

// stepHold runs the proportional position-hold controller.
// It computes the error between current position and hold target,
// and applies corrective velocity to cancel wind drift each tick.
func (s *Simulator) stepHold(dtSec float64) {
	mPerLon := metersPerDegreeLon(s.state.Lat)

	// Error in metres from current position to hold target.
	errX := (s.holdLon - s.state.Lon) * mPerLon            // east
	errY := (s.holdLat - s.state.Lat) * metersPerDegreeLat // north
	errZ := s.holdAlt - s.state.Alt                        // up

	// Proportional correction: stronger pull when further from hold point.
	cvx := errX * HoldProportionalGain
	cvy := errY * HoldProportionalGain
	cvz := errZ * HoldProportionalGain

	// Clamp to HoldMaxThrust so we don't oscillate wildly.
	cvx = clamp(cvx, -HoldMaxThrust, HoldMaxThrust)
	cvy = clamp(cvy, -HoldMaxThrust, HoldMaxThrust)
	cvz = clamp(cvz, -HoldMaxThrust, HoldMaxThrust)

	// Snapshot position before correction to compute actual net movement.
	prevLat, prevLon := s.state.Lat, s.state.Lon

	// Apply corrective displacement this tick.
	s.state.Lon += (cvx * dtSec) / mPerLon
	s.state.Lat += (cvy * dtSec) / metersPerDegreeLat
	s.state.Alt += cvz * dtSec

	// Vel shows the corrective thrust the drone is applying.
	// Speed shows ACTUAL net movement per second — near 0 when hold is stable
	// (thrust roughly cancels wind, so the drone barely moves).
	dxActual := (s.state.Lon - prevLon) * mPerLon
	dyActual := (s.state.Lat - prevLat) * metersPerDegreeLat
	s.state.Vel = Velocity{VX: cvx, VY: cvy, VZ: cvz}
	s.state.Speed = math.Sqrt(dxActual*dxActual+dyActual*dyActual) / dtSec
}

// moveToward moves the aircraft one dt step toward wp.
// Returns true when the aircraft has arrived within ArrivalTolerance.
func (s *Simulator) moveToward(wp *Waypoint, dtSec float64) bool {
	mPerLon := metersPerDegreeLon(s.state.Lat)

	dx := (wp.Lon - s.state.Lon) * mPerLon
	dy := (wp.Lat - s.state.Lat) * metersPerDegreeLat
	dz := wp.Alt - s.state.Alt

	horizDist := math.Sqrt(dx*dx + dy*dy)
	totalDist := math.Sqrt(dx*dx + dy*dy + dz*dz)

	if totalDist < ArrivalTolerance {
		s.state.Lat = wp.Lat
		s.state.Lon = wp.Lon
		s.state.Alt = wp.Alt
		s.state.Vel = Velocity{}
		s.state.Speed = 0
		return true
	}

	// Cruise speed for this leg.
	cruiseSpeed := DefaultCruiseSpeed
	if wp.Speed != nil && *wp.Speed > 0 {
		cruiseSpeed = *wp.Speed
	}

	// Proximity braking: linear ramp from cruiseSpeed → MinApproachSpeed
	// over the last BrakingDistance metres.
	speed := cruiseSpeed
	if totalDist < BrakingDistance {
		t := totalDist / BrakingDistance // 1.0 = full speed, 0.0 = at target
		speed = MinApproachSpeed + t*(cruiseSpeed-MinApproachSpeed)
	}

	// Clamp vertical component so it never exceeds DefaultClimbRate.
	if totalDist > 0 {
		vertFrac := math.Abs(dz) / totalDist
		if vertFrac*speed > DefaultClimbRate {
			speed = DefaultClimbRate / vertFrac
		}
	}

	stepDist := math.Min(speed*dtSec, totalDist)
	ratio := stepDist / totalDist

	s.state.Lon += (dx * ratio) / mPerLon
	s.state.Lat += (dy * ratio) / metersPerDegreeLat
	s.state.Alt += dz * ratio

	// Vel reflects own thrust only (wind added by environment separately).
	nx, ny, nz := dx/totalDist, dy/totalDist, dz/totalDist
	s.state.Vel = Velocity{VX: nx * speed, VY: ny * speed, VZ: nz * speed}
	s.state.Speed = math.Sqrt(nx*nx+ny*ny) * speed

	if horizDist > 0.01 {
		heading := math.Atan2(dx, dy) * 180.0 / math.Pi
		if heading < 0 {
			heading += 360.0
		}
		s.state.Heading = heading
	}

	return false
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
