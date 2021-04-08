#!/bin/bash
set -e

function kubectl() {
  .bin/minikube kubectl -- "$@"
}

function usage() {
  echo "Usage: deploy.sh --rmf-ws <path> --rmf-web-ws <path>"
}

options=$(getopt -o '' -l rmf-ws:,rmf-web-ws: -- "$@")
eval set -- "$options"
while true; do
  case "$1" in
    --rmf-ws)
      shift
      rmf_ws=$(realpath -s $1)
      ;;
    --rmf-web-ws)
      shift
      rmf_web_ws=$(realpath -s $1)
      ;;
    --)
      shift
      break
      ;;
  esac
  shift
done

if [[ -z $rmf_ws || -z $rmf_web_ws ]]; then
  usage
  exit 1
fi

cd $(dirname $0)

kubectl apply -f k8s/keycloak.yaml
echo 'waiting for keycloak to be ready...'
kubectl wait --for=condition=available deployment/keycloak

echo 'creating jwt configmap...'
node keycloak-tools/bootstrap-keycloak.js
node keycloak-tools/get-cert.js > keycloak.pem
openssl x509 -in keycloak.pem -pubkey -noout -out jwt-pub-key.pub
kubectl create configmap jwt-pub-key --from-file=jwt-pub-key.pub -o=yaml --dry-run=client | kubectl apply -f -

echo 'building base rmf image...'
docker build -t rmf-web/builder -f docker/builder.dockerfile $rmf_ws/src

echo 'building rmf-server image...'
docker build -t rmf-web/rmf-server -f docker/rmf-server.dockerfile $rmf_web_ws
echo 'publishing rmf-server image...'
docker save rmf-web/rmf-server | bash -c 'eval $(.bin/minikube docker-env) && docker load'
echo 'creating rmf-server configmap...'
kubectl create configmap rmf-server-config --from-file=rmf_server_config.py -o=yaml --dry-run=client | kubectl apply -f -
echo 'deploying rmf-server...'
.bin/minikube kubectl -- apply -f k8s/rmf-server.yaml

echo 'building dashboard image...'
docker build -t rmf-web/dashboard -f docker/dashboard.dockerfile $rmf_web_ws
echo 'publishing rmf-server image...'
docker save rmf-web/dashboard | bash -c 'eval $(.bin/minikube docker-env) && docker load'
echo 'deploying dashboard...'
.bin/minikube kubectl -- apply -f k8s/dashboard.yaml
