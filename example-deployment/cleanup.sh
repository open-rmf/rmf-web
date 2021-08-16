#!/bin/bash

# remove rmf-web images, kubernetes resources and stop minikube
# NOTE: This does not remove EVERYTHING, for example, non rmf-web images like ros, fluentd etc are not removed.
bash -c 'eval $(.bin/minikube docker-env) && docker system prune -f && docker rmi -f $(docker images -q "rmf-web/*") && docker system prune -f'
docker system prune -f && docker rmi -f $(docker images -q "rmf-web/*") && docker system prune -f

.bin/minikube kubectl -- delete all,pvc,sa -lproject=rmf-web-example-full
.bin/minikube stop
