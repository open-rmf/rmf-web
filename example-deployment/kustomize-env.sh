#!/bin/bash
set -e

kubectl kustomize "$1" | envsubst
