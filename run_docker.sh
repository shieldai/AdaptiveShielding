#!/bin/bash

DOCKER_ACC=mdei
DOCKER_REPO=shieldone
IMG_TAG=v1

CONTAINER_NAME=adaptive_shielding

DOCKER_IMG=$DOCKER_ACC/$DOCKER_REPO:$IMG_TAG

# docker build -t "$DOCKER_IMG" .
docker pull "$DOCKER_IMG"

# "access control disabled, clients can connect from any host"
xhost +

# ------ RUN A CONTAINER ---------

HOST_PATH=$(pwd)
DOCKER_PATH="/home/$(whoami)/$(basename "$HOST_PATH")"

docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOST_PATH/:$DOCKER_PATH/" \
    --workdir="$DOCKER_PATH/" \
    -e DISPLAY=$DISPLAY \
    "$DOCKER_IMG" \
    bash -c "./run.sh test"
