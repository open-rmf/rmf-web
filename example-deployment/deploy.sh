#!/bin/bash


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

docker build -t rmf-web/dashboard:$rmf_web_ver -f docker/dashboard.Dockerfile "$rmf_web_ws" \
  --build-arg BUILDER_TAG=$rmf_web_ver \
  --build-arg PUBLIC_URL='/dashboard' \
  --build-arg REACT_APP_TRAJECTORY_SERVER='ws://localhost:8006' \
  --build-arg REACT_APP_RMF_SERVER='https://example.com/rmf/api/v1' \
  --build-arg REACT_APP_AUTH_PROVIDER='keycloak' \
  --build-arg REACT_APP_KEYCLOAK_CONFIG='{ "realm": "rmf-web", "clientId": "dashboard", "url": "https://example.com/auth" }'
echo '📤 publishing dashboard image...'
docker save rmf-web/dashboard:$rmf_web_ver | bash -c 'eval $(.bin/minikube docker-env) && docker load'

docker build -t rmf-web/reporting-server:$rmf_web_ver -f docker/reporting-server.Dockerfile "$rmf_web_ws" --build-arg BUILDER_TAG=$rmf_web_ver
echo '📤 publishing reporting-server image...'
docker save rmf-web/reporting-server:$rmf_web_ver | bash -c 'eval $(.bin/minikube docker-env) && docker load'

docker build -t rmf-web/reporting:$rmf_web_ver -f docker/reporting.Dockerfile "$rmf_web_ws" \
  --build-arg BUILDER_TAG=$rmf_web_ver \
  --build-arg PUBLIC_URL='/reporting' \
  --build-arg REACT_APP_REPORTING_SERVER='https://example.com/logserver/api/v1' \
  --build-arg REACT_APP_AUTH_PROVIDER='keycloak' \
  --build-arg REACT_APP_KEYCLOAK_CONFIG='{ "realm": "rmf-web", "clientId": "reporting", "url": "https://example.com/auth" }'
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
try node keycloak-tools/get-cert.js > keycloak.pem
openssl x509 -in keycloak.pem -pubkey -noout -out jwt-pub-key.pub
kubectl create configmap jwt-pub-key --from-file=jwt-pub-key.pub -o=yaml --dry-run=client | kubectl apply -f -

echo 'deploying Minio...'
.bin/minikube kubectl -- apply -f k8s/minio.yaml

echo 'building base rmf image...'
docker build -t rmf-web/builder -f docker/builder.dockerfile $rmf_ws/src

echo 'building rmf-server image...'
docker build -t rmf-web/rmf-server -f docker/rmf-server.dockerfile $rmf_web_ws
echo 'publishing rmf-server image...'
docker save rmf-web/rmf-server | bash -c 'eval $(.bin/minikube docker-env) && docker load'
echo 'creating rmf-server configmap...'
kubectl create configmap rmf-server-config --from-file=rmf_server_config.py -o=yaml --dry-run=client | kubectl apply -f -
echo 'deploying rmf-server...'
kubectl apply -f k8s/rmf-server.yaml

echo 'building dashboard image...'
docker build -t rmf-web/dashboard -f docker/dashboard.dockerfile $rmf_web_ws
echo 'publishing dashboard image...'
docker save rmf-web/dashboard | bash -c 'eval $(.bin/minikube docker-env) && docker load'
echo 'deploying dashboard...'
kubectl apply -f k8s/dashboard.yaml

echo 'building minimal dashboard image...'
docker build -t rmf-web/minimal -f docker/minimal.dockerfile $rmf_web_ws
echo 'publishing minimal dashboard image...'
docker save rmf-web/minimal | bash -c 'eval $(.bin/minikube docker-env) && docker load'
echo 'deploying minimal dashboard...'
kubectl apply -f k8s/minimal.yaml

echo 'building reporting-server image...'
docker build -t rmf-web/reporting-server -f docker/reporting-server.dockerfile $rmf_web_ws
echo 'publishing reporting-server image...'
docker save rmf-web/reporting-server | bash -c 'eval $(.bin/minikube docker-env) && docker load'
echo 'creating reporting-server configmap...'
kubectl create configmap reporting-server-config --from-file=reporting_server_config.py -o=yaml --dry-run=client | kubectl apply -f -
echo 'deploying reporting-server...'
kubectl apply -f k8s/reporting-server.yaml


echo 'building reporting image...'
docker build -t rmf-web/reporting -f docker/reporting.dockerfile $rmf_web_ws
echo 'publishing reporting image...'
docker save rmf-web/reporting | bash -c 'eval $(.bin/minikube docker-env) && docker load'
echo 'deploying reporting-server...'
kubectl apply -f k8s/reporting.yaml

echo 'Applying FluentD configmap ...'
.bin/minikube kubectl -- apply -f k8s/fluentd-configmap.yaml
echo 'deploying FluentD daemonset...'
.bin/minikube kubectl -- apply -f k8s/fluentd.yaml

echo 'deploying cronjobs ...'
.bin/minikube kubectl -- apply -f k8s/cronjobs.yaml
try node keycloak-tools/get-cert.js > k8s/example-full/keycloak/keycloak.pem
openssl x509 -in k8s/example-full/keycloak/keycloak.pem -pubkey -noout -out k8s/example-full/keycloak/jwt-pub-key.pub
echo '✅ successfully setup keycloak'

# deploy all other components
export RMF_SERVER_TAG=$rmf_web_ver
export DASHBOARD_TAG=$rmf_web_ver
export REPORTING_SERVER_TAG=$rmf_web_ver
export REPORTING_TAG=$rmf_web_ver
./kustomize-env.sh k8s/example-full | kubectl apply -f -
