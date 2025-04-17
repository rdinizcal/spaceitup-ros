#!/usr/bin/env bash

ORG=openrobotics
IMAGE=space_robots_demo
TAG=latest

VCS_REF=""
VERSION=preview

# Exit script with failure if build fails
set -eo pipefail

# Paths
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DOCKER_DIR="${PROJECT_ROOT}/docker/space_robots"
DEMOS_SRC="${PROJECT_ROOT}/demos"
DEMOS_DST="${DOCKER_DIR}/tmp-demos"
MISSION_SRC="${PROJECT_ROOT}/mission_spec"
MISSION_DST="${DOCKER_DIR}/tmp-mission"

echo ""
echo "##### Preparing local demos folder #####"
echo ""

# Clean up old temp if exists
rm -rf "$DEMOS_DST"
rm -rf "$MISSION_DST"

# Copy local demos to build context
cp -r "$DEMOS_SRC" "$DEMOS_DST"
cp -r "$MISSION_SRC" "$MISSION_DST"

echo ""
echo "##### Building Space ROS Demo Docker Image #####"
echo ""

docker build -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" .

echo ""
echo "##### Cleaning up temporary demos folder #####"
echo ""

rm -rf "$DEMOS_DST"
rm -rf "$MISSION_DST"

echo ""
echo "##### Done! #####"