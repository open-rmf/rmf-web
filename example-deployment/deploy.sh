#!/bin/bash
set -e

if [ $# -lt 2 ]; then
  echo 'Usage: deploy.sh <rmf-ws> <rmf-web-ws>'
fi

here=$(realpath $(dirname $0))
rmf_ws=$(realpath $1)
rmf_web_ws=$(realpath $2)
cd "$here"

kubectl() {
  .bin/minikube kubectl -- "$@"
}
export -f kubectl

cd "$rmf_web_ws"
rmf_web_ver=$(git rev-parse HEAD)
cd "$here"

### build phase

docker build -t rmf-web/keycloak:12.04 -f docker/keycloak/keycloak.Dockerfile docker/keycloak/
echo '📤 publishing keycloak image...'
docker save rmf-web/keycloak:12.04 | bash -c 'eval $(.bin/minikube docker-env) && docker load'

./build-builder.sh "$rmf_web_ws" $rmf_web_ver

# NOTE: rmf-server depends on internal messages of rmf, but the tag of rmf-server is only based on the rmf-web version.
# This is because they are on different repos so we cannot pin the version according to the git commit hash.
# Also, rmf is a poly repo so there is no single commit hash we can use to uniquely identify the version.
./build-rmf-server.sh "$rmf_ws" "$rmf_web_ws" $rmf_web_ver
echo '📤 publishing rmf-server image...'
docker save rmf-web/rmf-server:$rmf_web_ver | bash -c 'eval $(.bin/minikube docker-env) && docker load'

docker build -t rmf-web/dashboard:$rmf_web_ver -f docker/dashboard.Dockerfile "$rmf_web_ws" --build-arg BUILDER_TAG=$rmf_web_ver
echo '📤 publishing dashboard image...'
docker save rmf-web/dashboard:$rmf_web_ver | bash -c 'eval $(.bin/minikube docker-env) && docker load'

docker build -t rmf-web/reporting-server:$rmf_web_ver -f docker/reporting-server.Dockerfile "$rmf_web_ws" --build-arg BUILDER_TAG=$rmf_web_ver
echo '📤 publishing reporting-server image...'
docker save rmf-web/reporting-server:$rmf_web_ver | bash -c 'eval $(.bin/minikube docker-env) && docker load'

docker build -t rmf-web/reporting:$rmf_web_ver -f docker/reporting.Dockerfile "$rmf_web_ws" --build-arg BUILDER_TAG=$rmf_web_ver
echo '📤 publishing reporting image...'
docker save rmf-web/reporting:$rmf_web_ver | bash -c 'eval $(.bin/minikube docker-env) && docker load'

### deploy phase

# need to deploy keycloak separately because we need to register the apps prior to generating
# the kustomization of the other deployments.
kubectl apply -k k8s/example-full/keycloak
echo '✅ deployed keycloak'

# this part only needs to be ran on the first deployment
echo '⏳ waiting for keycloak to be ready...'
kubectl wait --for=condition=available deployment/keycloak --timeout=2m
echo '🗒 setting up apps and users'
try() {
  "$@" || (sleep 1 && "$@") || (sleep 5 && "$@")
}
try node keycloak-tools/bootstrap-keycloak.js
try node keycloak-tools/get-cert.js > k8s/example-full/keycloak/keycloak.pem
openssl x509 -in k8s/example-full/keycloak/keycloak.pem -pubkey -noout -out k8s/example-full/keycloak/jwt-pub-key.pub
echo '✅ successfully setup keycloak'

# deploy all other components
export RMF_SERVER_TAG=$rmf_web_ver
export DASHBOARD_TAG=$rmf_web_ver
export REPORTING_SERVER_TAG=$rmf_web_ver
export REPORTING_TAG=$rmf_web_ver
./kustomize-env.sh k8s/example-full | kubectl apply -f -
