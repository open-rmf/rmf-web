# Fluentd on Kubernetes

## Pre-requisites

To create a kubernetes cluster in this example I used Kind.

To install kind (if you have ubuntu 20.04), you should follow these steps:
```
curl -Lo ./kind https://kind.sigs.k8s.io/dl/v0.10.0/kind-linux-amd64
chmod +x ./kind
mv ./kind /some-dir-in-your-PATH(maybe /usr/bin)/kind
```

You must install [kubectl](https://kubernetes.io/docs/tasks/tools/#kubectl) as well.

## Creating a Kubernetes cluster

Create a Kubernetes cluster using [kind](https://kind.sigs.k8s.io/docs/user/quick-start/)

```
kind create cluster --name fluentd --image kindest/node:v1.20.2
```

## Fluentd Manifests

Manifests from the official fluentd [github repo](https://github.com/fluent/fluentd-kubernetes-daemonset) . We used a custom one.

## Fluentd Docker

Official docker image of [fluentd](https://hub.docker.com/r/fluent/fluentd/). <br/>
We added some plugins so we are building our own.

The idea is to adjust `fluentd` to send logs to another destination (maybe a storage server like s3 but local).

To build the fluentd docker image

``` bash
cd packages/monitoring/kubernetes/dockerfile/
docker build . -t rmf/fluentd
```

## Fluentd Namespace

We should create its own namespace for this infrastructure component. <br/>

Let's create a `fluentd` namespace:

``` bash
kubectl create ns fluentd

```

## Fluentd Configmap

We have 4 files in our `fluentd-configmap.yaml` :
* `fluent.conf`: Our main config which includes all configurations we want to run.
* `pods-kind-fluent.conf`: `tail` config that sources all pod logs on the `kind` cluster. `kind` cluster writes its log in a different format than other clusters.
* `pods-fluent.conf`: `tail` config that sources all pod logs on the `kubernetes` host in the cloud. When running K8s in the cloud, logs may go into JSON format.
* `file-fluent.conf`: `match` config to capture all logs and write it to file for testing log collection </br>

Let's deploy our `configmap`:

``` bash
kubectl apply -f fluentd-configmap.yaml

```

## Fluentd Daemonset

Let's deploy the `daemonset`, on the kubernetes folder run:

``` bash
kubectl apply -f fluentd.yaml # to run the container
```

To check that the pod is running run:

``` bash
kubectl -n fluentd get pods
```

Let's deploy the example app that writes logs to `stdout`

``` bash
kubectl apply -f app-that-writes-logs.yaml
```

To check that the pod is running run:

``` bash
kubectl get pods
```