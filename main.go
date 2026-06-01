package main

import (
	"context"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/airborne/api"
	"github.com/airborne/simulator"
)

func main() {
	// ---- Initial aircraft state -----------------------------------------
	// Positioned at Ben Gurion International Airport, Israel.
	initial := simulator.AircraftState{
		Lat:       32.0055,
		Lon:       34.8854,
		Alt:       500.0,
		Mode:      simulator.ModeIdle,
		Timestamp: time.Now(),
	}

	// ---- Environment setup (optional; comment out layers as desired) -----
	env := simulator.MultiEnvironment{
		Envs: []simulator.Environment{
			simulator.WindEnvironment{WindVX: 5.0, WindVY: 2.0}, // 5 m/s east, 2 m/s north
			simulator.HumidityEnvironment{Factor: 0.95},         // 5% performance loss
			simulator.TerrainEnvironment{GroundElevation: 0.0},  // sea-level terrain floor
		},
	}

	// ---- Create and start simulator -------------------------------------
	sim := simulator.New(initial, env, simulator.DefaultTickRate)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	go sim.Run(ctx)

	// ---- HTTP server ----------------------------------------------------
	mux := http.NewServeMux()
	api.New(mux, sim.CmdChan, sim.StateReqChan, sim.EnvUpdateChan)

	// Serve map.html from the directory where the binary is run.
	mux.HandleFunc("GET /map", func(w http.ResponseWriter, r *http.Request) {
		http.ServeFile(w, r, "map.html")
	})

	addr := ":8080"
	srv := &http.Server{
		Addr:         addr,
		Handler:      mux,
		ReadTimeout:  10 * time.Second,
		WriteTimeout: 0, // 0 = no timeout (required for SSE streaming)
		IdleTimeout:  60 * time.Second,
	}

	// ---- Graceful shutdown on SIGINT / SIGTERM --------------------------
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)

	go func() {
		log.Printf("[main] listening on %s – map at http://localhost%s/map", addr, addr)
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("[main] server error: %v", err)
		}
	}()

	<-quit
	log.Println("[main] shutdown signal received")

	cancel() // stop simulator

	shutdownCtx, shutdownCancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer shutdownCancel()

	if err := srv.Shutdown(shutdownCtx); err != nil {
		log.Printf("[main] server shutdown error: %v", err)
	}
	log.Println("[main] clean shutdown complete")
}
