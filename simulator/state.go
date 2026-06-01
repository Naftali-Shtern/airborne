package simulator

import "time"

// Velocity holds the 3-axis velocity vector in m/s.
type Velocity struct {
	VX float64 // m/s east
	VY float64 // m/s north
	VZ float64 // m/s up
}

// EnvironmentSnapshot captures the current environment parameters for
// observability — included in every state/stream response.
type EnvironmentSnapshot struct {
	WindVX         float64 // eastward wind component (m/s)
	WindVY         float64 // northward wind component (m/s)
	WindVZ         float64 // vertical wind component (m/s)
	HumidityFactor float64 // speed multiplier [0,1]; 1.0 = no effect
	TerrainFloor   float64 // minimum safe altitude (ground + margin), metres ASL
}

// FlightMode describes what the aircraft is currently doing.
type FlightMode string

const (
	ModeIdle    FlightMode = "idle"    // no command, drifting with wind
	ModeFlying  FlightMode = "flying"  // actively navigating to a waypoint
	ModeHolding FlightMode = "holding" // position-hold loop active
	ModeStopped FlightMode = "stopped" // stopped, zero thrust, wind-drifting
)

// ActiveCommandSnapshot carries enough info for the map to draw waypoints/targets.
type ActiveCommandSnapshot struct {
	Type          string     // "goto" | "trajectory" | "hold" | "stop" | ""
	Target        *Waypoint  // set for goto and hold
	Waypoints     []Waypoint // set for trajectory
	Loop          bool       // trajectory loop flag
	WaypointIndex int        // current waypoint index in trajectory
}

// AircraftState is the complete observable state of the aircraft.
type AircraftState struct {
	Lat       float64               // degrees [-90, 90]
	Lon       float64               // degrees [-180, 180]
	Alt       float64               // meters above sea level
	Speed     float64               // ground speed magnitude in m/s (own thrust only)
	Heading   float64               // degrees [0, 360)
	Vel       Velocity              // velocity vector (own thrust only, wind excluded)
	Mode      FlightMode            // current flight mode
	Env       EnvironmentSnapshot   // live environment parameters
	Active    ActiveCommandSnapshot // current active command (for map display)
	Timestamp time.Time             // wall-clock time of this state snapshot
}

// CommandType distinguishes the four command kinds.
type CommandType int

const (
	CommandGoto CommandType = iota
	CommandTrajectory
	CommandStop
	CommandHold
)

func (ct CommandType) String() string {
	switch ct {
	case CommandGoto:
		return "goto"
	case CommandTrajectory:
		return "trajectory"
	case CommandStop:
		return "stop"
	case CommandHold:
		return "hold"
	default:
		return "unknown"
	}
}

// Waypoint is a single navigation target with an optional speed override.
type Waypoint struct {
	Lat   float64  // target latitude
	Lon   float64  // target longitude
	Alt   float64  // target altitude (m)
	Speed *float64 // nil = use simulator default cruise speed
}

// Command carries instructions from the API layer to the simulator goroutine.
type Command struct {
	Type      CommandType
	Target    *Waypoint  // used for CommandGoto
	Waypoints []Waypoint // used for CommandTrajectory
	Loop      bool       // trajectory looping flag
	IssuedAt  time.Time
}
