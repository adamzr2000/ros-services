#!/bin/bash

container_image="ros-humnle-dev"

echo "Building $container_image docker image."

sudo docker build . -t $container_image