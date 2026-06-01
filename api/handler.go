package api

import (
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"time"

	"github.com/airborne/simulator"
)

// Handler holds the channels needed to communicate with the simulator.
type Handler struct {
	CmdChan       chan simulator.Command
	StateReqChan  chan chan simulator.AircraftState
	EnvUpdateChan chan simulator.Environment
}

// New wires up all routes on the provided mux.
func New(mux *http.ServeMux, cmdChan chan simulator.Command, stateReqChan chan chan simulator.AircraftState, envUpdateChan chan simulator.Environment) *Handler {
	h := &Handler{
		CmdChan:       cmdChan,
		StateReqChan:  stateReqChan,
		EnvUpdateChan: envUpdateChan,
	}
	mux.HandleFunc("GET /health", h.Health)
	mux.HandleFunc("GET /state", h.State)
	mux.HandleFunc("GET /stream", h.Stream)
	mux.HandleFunc("POST /command/goto", h.Goto)
	mux.HandleFunc("POST /command/trajectory", h.Trajectory)
	mux.HandleFunc("POST /command/stop", h.Stop)
	mux.HandleFunc("POST /command/hold", h.Hold)
	mux.HandleFunc("POST /environment", h.UpdateEnvironment)
	return h
}

// ---- DTOs ---------------------------------------------------------------

type gotoRequest struct {
	Lat   float64  `json:"lat"`
	Lon   float64  `json:"lon"`
	Alt   float64  `json:"alt"`
	Speed *float64 `json:"speed,omitempty"`
}

type trajectoryRequest struct {
	Waypoints []gotoRequest `json:"waypoints"`
	Loop      bool          `json:"loop"`
}

// stateResponse is the full observable state returned by /state and /stream.
type stateResponse struct {
	// Position
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
	Alt float64 `json:"alt"`

	// Motion (own thrust only — wind drift excluded from Vel/Speed)
	Mode    string  `json:"mode"`     // idle | flying | holding | stopped
	Speed   float64 `json:"speed"`    // horizontal speed, m/s
	Speed3D float64 `json:"speed_3d"` // full 3-D speed including vz, m/s
	Heading float64 `json:"heading"`  // degrees clockwise from north [0,360)
	VX      float64 `json:"vx"`       // east  m/s
	VY      float64 `json:"vy"`       // north m/s
	VZ      float64 `json:"vz"`       // up    m/s

	// Live environment parameters
	WindVX         float64 `json:"wind_vx"`         // eastward wind, m/s
	WindVY         float64 `json:"wind_vy"`         // northward wind, m/s
	WindVZ         float64 `json:"wind_vz"`         // vertical wind, m/s
	HumidityFactor float64 `json:"humidity_factor"` // thrust multiplier [0,1]; 1=no effect
	TerrainFloor   float64 `json:"terrain_floor"`   // minimum safe altitude, m ASL

	Timestamp time.Time `json:"timestamp"`

	// Active command info for map rendering
	Active activeCommandResponse `json:"active"`
}

type waypointResponse struct {
	Lat   float64  `json:"lat"`
	Lon   float64  `json:"lon"`
	Alt   float64  `json:"alt"`
	Speed *float64 `json:"speed,omitempty"`
}

type activeCommandResponse struct {
	Type          string             `json:"type"` // "" | "goto" | "trajectory" | "hold"
	Target        *waypointResponse  `json:"target,omitempty"`
	Waypoints     []waypointResponse `json:"waypoints,omitempty"`
	Loop          bool               `json:"loop,omitempty"`
	WaypointIndex int                `json:"waypoint_index,omitempty"`
}

type healthResponse struct {
	Status string `json:"status"`
}

type errorResponse struct {
	Error string `json:"error"`
}

// ---- Helpers ------------------------------------------------------------

func writeJSON(w http.ResponseWriter, status int, v any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	_ = json.NewEncoder(w).Encode(v)
}

func writeError(w http.ResponseWriter, status int, msg string) {
	writeJSON(w, status, errorResponse{Error: msg})
}

func validateWaypoint(r gotoRequest) error {
	if r.Lat < -90 || r.Lat > 90 {
		return fmt.Errorf("lat must be in [-90, 90], got %v", r.Lat)
	}
	if r.Lon < -180 || r.Lon > 180 {
		return fmt.Errorf("lon must be in [-180, 180], got %v", r.Lon)
	}
	if r.Alt < 0 {
		return fmt.Errorf("alt must be >= 0, got %v", r.Alt)
	}
	if r.Speed != nil && (*r.Speed <= 0 || *r.Speed > 1000) {
		return fmt.Errorf("speed must be in (0, 1000] m/s, got %v", *r.Speed)
	}
	return nil
}

func toWaypoint(r gotoRequest) simulator.Waypoint {
	return simulator.Waypoint{Lat: r.Lat, Lon: r.Lon, Alt: r.Alt, Speed: r.Speed}
}

func toWaypointResponse(wp simulator.Waypoint) waypointResponse {
	return waypointResponse{Lat: wp.Lat, Lon: wp.Lon, Alt: wp.Alt, Speed: wp.Speed}
}

func toActiveResponse(a simulator.ActiveCommandSnapshot) activeCommandResponse {
	resp := activeCommandResponse{Type: a.Type, Loop: a.Loop, WaypointIndex: a.WaypointIndex}
	if a.Target != nil {
		wr := toWaypointResponse(*a.Target)
		resp.Target = &wr
	}
	if len(a.Waypoints) > 0 {
		resp.Waypoints = make([]waypointResponse, len(a.Waypoints))
		for i, wp := range a.Waypoints {
			resp.Waypoints[i] = toWaypointResponse(wp)
		}
	}
	return resp
}

func toStateResponse(st simulator.AircraftState) stateResponse {
	return stateResponse{
		Lat:            st.Lat,
		Lon:            st.Lon,
		Alt:            st.Alt,
		Mode:           string(st.Mode),
		Speed:          st.Speed,
		Speed3D:        math.Sqrt(st.Vel.VX*st.Vel.VX + st.Vel.VY*st.Vel.VY + st.Vel.VZ*st.Vel.VZ),
		Heading:        st.Heading,
		VX:             st.Vel.VX,
		VY:             st.Vel.VY,
		VZ:             st.Vel.VZ,
		WindVX:         st.Env.WindVX,
		WindVY:         st.Env.WindVY,
		WindVZ:         st.Env.WindVZ,
		HumidityFactor: st.Env.HumidityFactor,
		TerrainFloor:   st.Env.TerrainFloor,
		Timestamp:      st.Timestamp,
		Active:         toActiveResponse(st.Active),
	}
}

func (h *Handler) getState() simulator.AircraftState {
	respChan := make(chan simulator.AircraftState, 1)
	h.StateReqChan <- respChan
	return <-respChan
}

func (h *Handler) sendCommand(cmd simulator.Command) {
	h.CmdChan <- cmd
}

// ---- Endpoints ----------------------------------------------------------

// GET /health
func (h *Handler) Health(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, healthResponse{Status: "ok"})
}

// GET /state
func (h *Handler) State(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, toStateResponse(h.getState()))
}

// POST /command/goto
func (h *Handler) Goto(w http.ResponseWriter, r *http.Request) {
	var req gotoRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		writeError(w, http.StatusBadRequest, "invalid JSON: "+err.Error())
		return
	}
	if err := validateWaypoint(req); err != nil {
		writeError(w, http.StatusUnprocessableEntity, err.Error())
		return
	}
	wp := toWaypoint(req)
	speedStr := "default"
	if req.Speed != nil {
		speedStr = fmt.Sprintf("%.0f m/s", *req.Speed)
	}
	log.Printf("[api] goto lat=%.4f lon=%.4f alt=%.1f speed=%s", wp.Lat, wp.Lon, wp.Alt, speedStr)
	h.sendCommand(simulator.Command{Type: simulator.CommandGoto, Target: &wp, IssuedAt: time.Now()})
	writeJSON(w, http.StatusAccepted, map[string]string{"status": "accepted"})
}

// POST /command/trajectory
func (h *Handler) Trajectory(w http.ResponseWriter, r *http.Request) {
	var req trajectoryRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		writeError(w, http.StatusBadRequest, "invalid JSON: "+err.Error())
		return
	}
	if len(req.Waypoints) == 0 {
		writeError(w, http.StatusUnprocessableEntity, "waypoints must not be empty")
		return
	}
	waypoints := make([]simulator.Waypoint, 0, len(req.Waypoints))
	for i, wp := range req.Waypoints {
		if err := validateWaypoint(wp); err != nil {
			writeError(w, http.StatusUnprocessableEntity, fmt.Sprintf("waypoint[%d]: %s", i, err.Error()))
			return
		}
		waypoints = append(waypoints, toWaypoint(wp))
	}
	log.Printf("[api] trajectory waypoints=%d loop=%v", len(waypoints), req.Loop)
	h.sendCommand(simulator.Command{Type: simulator.CommandTrajectory, Waypoints: waypoints, Loop: req.Loop, IssuedAt: time.Now()})
	writeJSON(w, http.StatusAccepted, map[string]string{"status": "accepted"})
}

// POST /command/stop — zero thrust, aircraft drifts freely with wind.
func (h *Handler) Stop(w http.ResponseWriter, r *http.Request) {
	log.Println("[api] stop")
	h.sendCommand(simulator.Command{Type: simulator.CommandStop, IssuedAt: time.Now()})
	writeJSON(w, http.StatusAccepted, map[string]string{"status": "accepted"})
}

// POST /command/hold — engage position-hold loop to actively counter wind.
func (h *Handler) Hold(w http.ResponseWriter, r *http.Request) {
	log.Println("[api] hold")
	h.sendCommand(simulator.Command{Type: simulator.CommandHold, IssuedAt: time.Now()})
	writeJSON(w, http.StatusAccepted, map[string]string{"status": "accepted"})
}

// ---- Environment update DTOs --------------------------------------------

type windConfig struct {
	VX float64 `json:"vx"`
	VY float64 `json:"vy"`
	VZ float64 `json:"vz"`
}

type humidityConfig struct {
	Factor float64 `json:"factor"`
}

type terrainConfig struct {
	GroundElevation float64 `json:"ground_elevation"`
}

// environmentRequest lets callers enable, reconfigure, or disable individual
// environment layers at runtime. Omit a field entirely to disable that layer.
type environmentRequest struct {
	Wind     *windConfig     `json:"wind,omitempty"`
	Humidity *humidityConfig `json:"humidity,omitempty"`
	Terrain  *terrainConfig  `json:"terrain,omitempty"`
}

// POST /environment — replace the active environment configuration.
// Each layer is optional; omitting it disables that effect.
//
// Examples:
//
//	{"wind":{"vx":10,"vy":0,"vz":0}}                        → only wind
//	{"wind":{"vx":5,"vy":2},"humidity":{"factor":0.9}}       → wind + humidity
//	{}                                                        → all effects off
func (h *Handler) UpdateEnvironment(w http.ResponseWriter, r *http.Request) {
	var req environmentRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		writeError(w, http.StatusBadRequest, "invalid JSON: "+err.Error())
		return
	}

	var envs []simulator.Environment

	if req.Wind != nil {
		envs = append(envs, simulator.WindEnvironment{
			WindVX: req.Wind.VX, WindVY: req.Wind.VY, WindVZ: req.Wind.VZ,
		})
	}
	if req.Humidity != nil {
		if req.Humidity.Factor <= 0 || req.Humidity.Factor > 1 {
			writeError(w, http.StatusUnprocessableEntity, "humidity factor must be in (0, 1]")
			return
		}
		envs = append(envs, simulator.HumidityEnvironment{Factor: req.Humidity.Factor})
	}
	if req.Terrain != nil {
		if req.Terrain.GroundElevation < 0 {
			writeError(w, http.StatusUnprocessableEntity, "ground_elevation must be >= 0")
			return
		}
		envs = append(envs, simulator.TerrainEnvironment{GroundElevation: req.Terrain.GroundElevation})
	}

	log.Printf("[api] environment update wind=%v humidity=%v terrain=%v",
		req.Wind, req.Humidity, req.Terrain)
	h.EnvUpdateChan <- simulator.MultiEnvironment{Envs: envs}
	writeJSON(w, http.StatusAccepted, map[string]string{"status": "accepted"})
}

// GET /stream — Server-Sent Events at ~10 Hz.
func (h *Handler) Stream(w http.ResponseWriter, r *http.Request) {
	flusher, ok := w.(http.Flusher)
	if !ok {
		writeError(w, http.StatusInternalServerError, "streaming not supported")
		return
	}
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "keep-alive")
	w.Header().Set("X-Accel-Buffering", "no")

	log.Println("[api] SSE client connected")
	defer log.Println("[api] SSE client disconnected")

	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-r.Context().Done():
			return
		case <-ticker.C:
			payload, err := json.Marshal(toStateResponse(h.getState()))
			if err != nil {
				continue
			}
			fmt.Fprintf(w, "data: %s\n\n", payload)
			flusher.Flush()
		}
	}
}
