#!/bin/bash

container_image="go2-test"

echo "Building $container_image docker image."
sudo docker build . -t $container_image