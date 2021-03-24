# Deploying

The below snippets should be run from the server.

## Start the server
```bash
./start-server.sh
```
This will require internet connection. This will download docker and minikube and start a kubernetes cluster.

## Build
```bash
./prepare-rmf-web.sh
```
You need to be connected to the internet for this.

## Deploy
```bash
./deploy-rmf-web.sh
```
You need to be connected to the dp3 network, you don't need to be connected to the internet anymore.

# Accessing the dashboard
Since we don't have a dns server, you need to add the ip of the server to `/etc/hosts`.
```bash
echo '<ip-of-server> rmf.com' | sudo tee /etc/hosts
```
**Note:** this should be ran from the client.

Open a browser and go to `https://rmf.com/dashboard`, you may get a security warning saying that the connection is not private, this is normal since we are using a self-signed cert, it is (relatively) safe to ignore the warning.

When greeted with a login prompt, use `user=dp3 password=dp3`.

# Stopping the server
```bash
./stop-server.sh
```

# Troubleshooting

## The page failed to load

1. Make sure `rmf.com` is in your hosts try, try `ping rmf.com` to check if it is reachable.
2. Make sure all the pods are working, `.bin/minikube kubectl -- get pods` should show 3 pods all RUNNING.

## I got a blank white page

1. Make sure all the pods are working, `.bin/minikube kubectl -- get pods` should show 3 pods all RUNNING.

## It is stuck in "downloading building map..."

1. Make sure rmf is deployed and contactable, use `ros2 topic echo ...` to check connectivity (be sure to do this on the server, not the client!).

## I need to make changes in production

