#!/bin/bash
# ==============================================================================
#  Smart Wheelchair Automated Multi-Repo & Submodule Release Sync Script
#  Synchronizes development branches to stable target branches:
#   1. wheelchair_intelligence : development -> main
#   2. glass_detection         : development -> main
#   3. wheelchair2            : dev_deployment -> deployment (with submodules)
#   4. wheelchair_frontend     : development -> main
# ==============================================================================

set -e # Exit immediately on error

echo "======================================================================"
echo "  STARTING AUTOMATED RELEASE BRANCH SYNCHRONIZATION"
echo "======================================================================"

# ── 1. Synchronize wheelchair_intelligence (development -> main) ───────
INTEL_DIR="/home/ducky/wheelchair_ws/wheelchair_intelligence/main"
if [ -d "$INTEL_DIR" ]; then
    echo -e "\n[1/4] Syncing wheelchair_intelligence (development -> main)..."
    cd "$INTEL_DIR"
    git fetch origin
    #git checkout main 2>/dev/null || git checkout -b main origin/main
    git reset --hard origin/development
    git push origin main --force
    #git checkout development
    echo "  wheelchair_intelligence main branch is 1:1 with development"
fi

# ── 2. Synchronize glass_detection (development -> main) ─────────────
GLASS_DIR="/home/ducky/wheelchair_ws/glass_detection"
if [ -d "$GLASS_DIR" ]; then
    echo -e "\n[2/4] Syncing glass_detection (development -> main)..."
    cd "$GLASS_DIR"
    git fetch origin
    git checkout main 2>/dev/null || git checkout -b main origin/main
    git reset --hard origin/development
    git push origin main --force
    git checkout development
    echo " glass_detection main branch is 1:1 with development"
fi

# ── 3. Synchronize wheelchair2 & Submodules (dev_deployment -> deployment)
WC2_DIR="/home/robot/wheelchair_ws/wheelchair2"
if [ -d "$WC2_DIR" ]; then
    echo -e "\n[3/4] Syncing wheelchair2 & submodules (dev_deployment -> deployment)..."
    cd "$WC2_DIR"
    git fetch origin
    git checkout deployment 2>/dev/null || git checkout -b deployment origin/deployment
    git reset --hard origin/dev_deployment
    
    # Update and sync all git submodules
    git submodule update --init --recursive
    
    git push origin deployment --force
    git checkout dev_deployment
    echo "  wheelchair2 deployment branch & submodules are 1:1 with dev_deployment"
fi

# ── 4. Synchronize wheelchair_frontend (development -> main) ─────────
FRONTEND_DIR="/home/robot/wheelchair_ws/wheelchair_frontend"
if [ -d "$FRONTEND_DIR" ]; then
    echo -e "\n[4/4] Syncing wheelchair_frontend (development -> main)..."
    cd "$FRONTEND_DIR"
    git fetch origin
    git checkout main 2>/dev/null || git checkout -b main origin/main
    git reset --hard origin/development
    git push origin main --force
    git checkout development
    echo "  wheelchair_frontend main branch is 1:1 with development"
fi

echo -e "\n======================================================================"
echo "  SUCCESS: ALL PRODUCTION BRANCHES & SUBMODULES SYNCHRONIZED!"
echo "======================================================================"