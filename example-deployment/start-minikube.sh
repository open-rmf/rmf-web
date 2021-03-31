#!/bin/bash
set -e

cd $(dirname $0)
. ./bash-functions.sh

# install docker
if [[ -z $(which docker) ]]; then
  print '⚠️  docker not found! Attempting to install it with apt.'
  sudo apt update && sudo apt install -y docker.io
fi

# install minikube
if ! test -f .bin/minikube; then
  print '⚠️  minikube not found! Downloading it to ".bin/minikube".'
  mkdir -p .bin
  curl -L https://storage.googleapis.com/minikube/releases/latest/minikube-linux-amd64 -o .bin/minikube
  chmod +x .bin/minikube
fi

# start minikube
.bin/minikube start --cpus=$(nproc) --driver=docker --addons ingress
