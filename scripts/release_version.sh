#!/bin/bash
# ==============================================================================
#  Smart Wheelchair Semantic Version Release Script
#  Usage: ./release_version.sh <version_tag> [release_notes]
#  Example: ./release_version.sh v1.2.0 "Release v1.2.0: Integrated battery telemetry and Tailscale Funnel"
# ==============================================================================

set -e

VERSION_TAG="$1"
RELEASE_NOTES="${2:-Official release $VERSION_TAG}"

if [ -z "$VERSION_TAG" ]; then
    echo "ERROR: Version tag is required!"
    echo "Usage: $0 <version_tag> [release_notes]"
    echo "Example: $0 v1.2.0 'Release v1.2.0: Battery telemetry & Tailscale Funnel'"
    exit 1
fi

echo "======================================================================"
echo "  STARTING AUTOMATED VERSION RELEASE: $VERSION_TAG"
echo "======================================================================"
echo "  Release Notes: $RELEASE_NOTES"
echo "──────────────────────────────────────────────────────────────────────"

# Repositories mapping
REPOS=(
    "wheelchair_frontend:/home/robot/wheelchair_ws/wheelchair_frontend:development"
    "wheelchair_intelligence:/home/robot/wheelchair_ws/google_quant/wheelchair_intelligence/development:development"
    "glass_detection:/home/robot/wheelchair_ws/glass_detection:development"
    "wheelchair2:/home/robot/wheelchair_ws/wheelchair2:dev_deployment"
)

MANIFEST_FILE="/home/robot/wheelchair_ws/RELEASE_MANIFEST_${VERSION_TAG}.md"
echo "# Smart Wheelchair Release Manifest - ${VERSION_TAG}" > "$MANIFEST_FILE"
echo "" >> "$MANIFEST_FILE"
echo "**Release Date**: $(date '+%Y-%m-%d %H:%M:%S %Z')" >> "$MANIFEST_FILE"
echo "**Release Notes**: ${RELEASE_NOTES}" >> "$MANIFEST_FILE"
echo "" >> "$MANIFEST_FILE"
echo "## Component Release Details" >> "$MANIFEST_FILE"
echo "" >> "$MANIFEST_FILE"
echo "| Repository | Branch | Commit SHA | Tag | Status |" >> "$MANIFEST_FILE"
echo "| :--- | :--- | :--- | :--- | :--- |" >> "$MANIFEST_FILE"

# Tag each repository
for item in "${REPOS[@]}"; do
    IFS=":" read -r REPO_NAME REPO_PATH REPO_BRANCH <<< "$item"
    
    if [ -d "$REPO_PATH" ]; then
        echo -e "\n Tagging ${REPO_NAME} (${REPO_BRANCH})..."
        cd "$REPO_PATH"
        
        # Ensure clean state and fetch latest
        git fetch origin
        git checkout "$REPO_BRANCH"
        
        # Check if tag already exists
        if git rev-parse "$VERSION_TAG" >/dev/null 2>&1; then
            echo "   Tag $VERSION_TAG already exists in $REPO_NAME. Updating tag..."
            git tag -d "$VERSION_TAG" 2>/dev/null || true
            git push origin ":refs/tags/$VERSION_TAG" 2>/dev/null || true
        fi
        
        # Create annotated tag
        git tag -a "$VERSION_TAG" -m "$RELEASE_NOTES"
        
        # Push tag to GitHub
        git push origin "$VERSION_TAG"
        
        COMMIT_SHA=$(git rev-parse --short HEAD)
        echo "| \`${REPO_NAME}\` | \`${REPO_BRANCH}\` | \`${COMMIT_SHA}\` | \`${VERSION_TAG}\` |  Tagged & Pushed |" >> "$MANIFEST_FILE"
        echo "  ✓ $REPO_NAME tagged at $COMMIT_SHA and pushed to GitHub"
    else
        echo "   Directory $REPO_PATH not found. Skipping."
    fi
done

echo -e "\n======================================================================"
echo "  VERSION RELEASE ${VERSION_TAG} COMPLETED SUCCESSFULLY!"
echo "======================================================================"
echo "  Manifest generated: ${MANIFEST_FILE}"
echo "======================================================================"
cat "$MANIFEST_FILE"
