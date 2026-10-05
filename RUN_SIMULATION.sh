#!/bin/bash
# TSYP Simulation Phase 1 - Run Script
# This script demonstrates how to run the fully functional simulation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔════════════════════════════════════════════════════════════════════════════════╗"
echo "║                                                                                ║"
echo "║                   TSYP SIMULATION PHASE 1 - COMPLETE & READY                  ║"
echo "║                                                                                ║"
echo "║  Living Map Scenario: Writer Robot + Executor Robot + Beacon Network + Gateway ║"
echo "║                                                                                ║"
echo "╚════════════════════════════════════════════════════════════════════════════════╝"
echo ""

# Show available options
echo "Available commands:"
echo ""
echo "  1. Run scenario (headless, logs to terminal)"
echo "     $ ./RUN_SIMULATION.sh run"
echo ""
echo "  2. Run scenario and save logs to file"
echo "     $ ./RUN_SIMULATION.sh run-save"
echo ""
echo "  3. Build Docker image (fresh)"
echo "     $ ./RUN_SIMULATION.sh build"
echo ""
echo "  4. Run with Gazebo GUI (requires X11 forwarding)"
echo "     $ ./RUN_SIMULATION.sh gui"
echo ""
echo "  5. Clean up Docker containers"
echo "     $ ./RUN_SIMULATION.sh clean"
echo ""
echo "  6. Check specific topics during run"
echo "     $ ./RUN_SIMULATION.sh topics"
echo ""
echo "─────────────────────────────────────────────────────────────────────────────────"
echo ""

if [ $# -eq 0 ]; then
    echo "No command provided. Use one of the commands above."
    exit 0
fi

case "$1" in
    run)
        echo "▶ Starting scenario (150s timeout, headless mode)..."
        echo ""
        timeout 150s docker compose run --rm tsyp-simulation \
            ros2 launch tsyp_simulation scenario_launch.py
        echo ""
        echo "✓ Scenario complete!"
        ;;
    
    run-save)
        OUTPUT_FILE="scenario-run-$(date +%Y%m%d-%H%M%S).log"
        echo "▶ Starting scenario and saving to: $OUTPUT_FILE"
        echo ""
        timeout 150s docker compose run --rm tsyp-simulation \
            ros2 launch tsyp_simulation scenario_launch.py 2>&1 | tee "$OUTPUT_FILE"
        echo ""
        echo "✓ Scenario complete! Logs saved to: $OUTPUT_FILE"
        echo ""
        echo "Quick event summary:"
        grep -E "PHYSICAL DROP|EVENT BEACON WRITE|Gateway dispatch|Executor reached|Executor report" "$OUTPUT_FILE" | head -20
        ;;
    
    build)
        echo "▶ Building Docker image (fresh, no cache)..."
        echo ""
        docker compose build --no-cache tsyp-simulation
        echo ""
        echo "✓ Docker image built successfully!"
        ;;
    
    gui)
        echo "▶ Starting scenario with Gazebo GUI..."
        echo "(Requires X11 forwarding; use Ctrl+C to stop)"
        echo ""
        GZ_GUI=1 docker compose up tsyp-simulation || true
        echo ""
        echo "▶ Cleaning up containers..."
        docker compose down --remove-orphans
        echo "✓ Done!"
        ;;
    
    clean)
        echo "▶ Stopping and removing all containers..."
        docker compose down --remove-orphans
        echo ""
        echo "✓ Cleanup complete!"
        ;;
    
    topics)
        echo "▶ Starting scenario in background for topic inspection..."
        timeout 30s docker compose run --rm tsyp-simulation \
            ros2 launch tsyp_simulation scenario_launch.py >/dev/null 2>&1 &
        
        sleep 10
        echo ""
        echo "Writer Robot Topics:"
        docker compose run --rm tsyp-simulation \
            ros2 topic list 2>/dev/null | grep writer_robot | head -10
        echo ""
        echo "Executor Robot Topics:"
        docker compose run --rm tsyp-simulation \
            ros2 topic list 2>/dev/null | grep executor_robot | head -10
        echo ""
        echo "Network Topics:"
        docker compose run --rm tsyp-simulation \
            ros2 topic list 2>/dev/null | grep "^/network" | head -10
        echo ""
        ;;
    
    *)
        echo "Unknown command: $1"
        echo "Use 'run', 'run-save', 'build', 'gui', 'clean', or 'topics'"
        exit 1
        ;;
esac
