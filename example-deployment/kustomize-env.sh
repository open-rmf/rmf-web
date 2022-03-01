#!/bin/bash
set -e

kubectl() {
  .bin/minikube kubectl -- "$@"
}
export -f kubectl

kubectl kustomize "$1" | envsubst
