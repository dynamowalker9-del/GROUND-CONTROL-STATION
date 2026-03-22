#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
#  DroneGuard — Quick-start script
#  Run this once to install deps then launch the backend server.
# ─────────────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo "  ██████╗ ██████╗  ██████╗ ███╗   ██╗███████╗"
echo "  ██╔══██╗██╔══██╗██╔═══██╗████╗  ██║██╔════╝"
echo "  ██║  ██║██████╔╝██║   ██║██╔██╗ ██║█████╗  "
echo "  ██║  ██║██╔══██╗██║   ██║██║╚██╗██║██╔══╝  "
echo "  ██████╔╝██║  ██║╚██████╔╝██║ ╚████║███████╗"
echo "  ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝"
echo "  DroneGuard MAVLink Backend  v1.0"
echo ""

# ── check python ──────────────────────────────────────────────────────────────
if ! command -v python3 &>/dev/null; then
    echo "❌  python3 not found. Install Python 3.9+ first."
    exit 1
fi

PYTHON=$(command -v python3)
echo "✅  Python: $($PYTHON --version)"

# ── virtual env (optional but clean) ─────────────────────────────────────────
if [ ! -d ".venv" ]; then
    echo "🔧  Creating virtual environment (.venv)..."
    $PYTHON -m venv .venv
fi

source .venv/bin/activate
echo "✅  Virtual env active"

# ── install deps ──────────────────────────────────────────────────────────────
echo "🔧  Installing dependencies..."
pip install --quiet --upgrade pip
pip install --quiet -r requirements.txt
echo "✅  Dependencies installed"

# ── launch ────────────────────────────────────────────────────────────────────
HOST="${DRONEGUARD_HOST:-0.0.0.0}"
PORT="${DRONEGUARD_PORT:-8765}"

echo ""
echo "🚀  Starting DroneGuard server on ws://$HOST:$PORT"
echo "    Open drone_gas_monitor.html in your browser"
echo "    Press Ctrl+C to stop"
echo ""

$PYTHON server.py --host "$HOST" --port "$PORT"
