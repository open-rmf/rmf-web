[![Nightly](https://github.com/osrf/rmf-web/workflows/Nightly/badge.svg)](https://github.com/osrf/rmf-web/actions?query=workflow%3ANightly)
[![codecov](https://codecov.io/gh/osrf/rmf-web/branch/main/graph/badge.svg)](https://codecov.io/gh/osrf/rmf-web)

# Running the Dashboard

## Prerequisites

### Ubuntu 20.04

Install docker and docker-compose
```bash
sudo apt update && sudo apt install docker.io docker-compose
```

Install nodejs
```bash
sudo apt update && sudo apt install curl
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.2/install.sh | bash
nvm install 12
```

Install rmf
```bash
curl -sL http://rmf.servehttp.com/repos.key | sudo apt-key add -
echo 'deb [arch=amd64] http://rmf.servehttp.com/ubuntu/main focal main' | sudo tee /etc/apt/sources.list.d/rmf.list
sudo apt update && sudo apt install '^ros-foxy-rmf-.*'
```

### Others

Refer to the following documentations:

* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker-compose](https://docs.docker.com/compose/install/)
* [nodejs](https://nodejs.org/en/download/package-manager/)
  * alternative: [nvm](https://github.com/nvm-sh/nvm)
* [rmf_core](https://github.com/osrf/rmf_core)
* [traffic_editor](https://github.com/osrf/traffic_editor)
* [rmf schedule visualizer](https://github.com/osrf/rmf_schedule_visualizer)
* [rmf_demos](https://github.com/osrf/rmf_demos)

## Bootstrap
Before running the commands, make sure that rmf is sourced.
```bash
git clone https://github.com/osrf/rmf-web
cd rmf-web
npm run bootstrap
```

## Launching
Before running the commands, make sure that rmf is sourced.
```bash
cd packages/dashboard
npm start
```
When presented with a login screen, use `user=admin password=admin`

## Troubleshooting
First thing to try is to build rmf from source, the dashboard currently does not guarantee support for different versions of rmf. Refer to each of the rmf repos for instruction to build them.

# Further documentation

Please see the [README](packages/dashboard/README.md) in the dashboard package.
