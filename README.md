[![Nightly](https://github.com/open-rmf/rmf-web/actions/workflows/nightly.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/nightly.yml) [![Dashboard End-to-End](https://github.com/open-rmf/rmf-web/actions/workflows/dashboard-e2e.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/dashboard-e2e.yml) [![Reporting End-to-End](https://github.com/open-rmf/rmf-web/actions/workflows/reporting-e2e.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/reporting-e2e.yml) [![react-components](https://github.com/open-rmf/rmf-web/workflows/react-components/badge.svg)](https://github.com/open-rmf/rmf-web/actions?query=workflow%3Areact-components+branch%3Amain) [![dashboard](https://github.com/open-rmf/rmf-web/workflows/dashboard/badge.svg)](https://github.com/open-rmf/rmf-web/actions?query=workflow%3Adashboard+branch%3Amain) [![api-server](https://github.com/open-rmf/rmf-web/workflows/api-server/badge.svg)](https://github.com/open-rmf/rmf-web/actions?query=workflow%3Aapi-server+branch%3Amain) [![reporting](https://github.com/open-rmf/rmf-web/actions/workflows/reporting.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/reporting.yml) [![reporting-server](https://github.com/open-rmf/rmf-web/actions/workflows/reporting-server.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/reporting-server.yml) [![rmf-auth](https://github.com/open-rmf/rmf-web/actions/workflows/rmf-auth.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/rmf-auth.yml) [![ros-translator](https://github.com/open-rmf/rmf-web/actions/workflows/ros-translator.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/ros-translator.yml) [![codecov](https://codecov.io/gh/open-rmf/rmf-web/branch/main/graph/badge.svg)](https://codecov.io/gh/open-rmf/rmf-web)

# Building the Dashboard

## Prerequisites

### Ubuntu 20.04

Install nodejs
```bash
sudo apt update && sudo apt install curl
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.2/install.sh | bash
nvm install 14
```

A recent version of pipenv is needed, the system packaged version is too old.
```bash
pip3 install pipenv
```

### Install rmf

Refer to the following documentation:

* [rmf](https://github.com/open-rmf/rmf)

### Others

Refer to the following documentations:

* [nodejs](https://nodejs.org/en/download/package-manager/) >= 12
* [rmf_demos](https://github.com/open-rmf/rmf_demos)

## Bootstrap

Before running the commands, make sure that rmf is sourced. We recommend using an `npm` version lower than 7.0.0 ([more information](https://github.com/open-rmf/rmf-web/issues/232)).
```bash
git clone https://github.com/open-rmf/rmf-web
cd rmf-web
npm install -g lerna@4
lerna bootstrap
```

You may also choose to bootstrap only the dashboard
```bash
lerna bootstrap --scope=rmf-dashboard
```

### Keycloak
If you would like to test out authentication and authorization locally, you will need to install and set up Keycloak. The defaults are for Keycloak to be listening on 127.0.0.1:8080 with:
* An administrator with username `admin` and password `admin`
* A user with username `admin` and password `admin`
* A user with username `example` and password `example`

The JWT Public Key for rmf-server authentication configuration will be saved in `/tmp/jwt-pub-key.pub`.

#### Docker
We can use Docker to quickly bring up a Keycloak instance.

Install docker: `https://docs.docker.com/engine/install/ubuntu/`

start a a database instance: `docker run -it --rm --name rmf-keycloak --network=host -e KEYCLOAK_USER=admin -e KEYCLOAK_PASSWORD=admin quay.io/keycloak/keycloak`

to stop the instance: `docker kill rmf-keycloak

Then, run the following script to bootstrap Keycloak with default configurations:
```
bash scripts/bootstrap-keycloak.bash
```

#### Bare Metal
The Keycloak executables can be found in the `rmf-web` root folder in the hidden holder `.keycloak`.
You can run the following script to set up a local Keycloak instance
```
bash scripts/setup-local-keycloak.bash  
```

Then, run the following script to bootstrap Keycloak with default configurations:
```
bash scripts/bootstrap-keycloak.bash
```

#### Issues
If you get an error "Signature Verification Failed" on the rmf-server, try the following:
* Terminate all instances of the dashboard client currently open
* Terminate all instances of api-server and keycloak currently running
* Delete `.keycloak` folder
* Relaunch everything

## Launching

Before running the commands, make sure that rmf is sourced.
```bash
cd packages/dashboard
npm start
```
When presented with a login screen, use `user=admin password=admin`.

This launches a development server with the office world from `rmf_demos`. The server is useful for development but is obviously not useful for actual usage.

## Configuration

See the [rmf-dashboard](packages/dashboard/README.md#configuration) docs.

## Troubleshooting
First thing to try is to build rmf from source, in order to speed up development, `rmf-web` may use in-development features of rmf. That means that the binary releases may not have the features required, sometimes the features `rmf-web` uses may be so new that not even the rolling releases has it.

Refer to [rmf_demos](https://github.com/open-rmf/rmf_demos) for instructions to build rmf. You should end up with a colcon workspace with all of rmf packages, remember to source the workspace before running any of the commands.

## Deploying

See [example deployment](example-deployment/README.md)

## Developing

### About lerna

This repo uses [lerna](https://github.com/lerna/lerna) to manage the packages. As such, you would not run npm commands with some side effects

  * commands that manipulates the packages, e.g., `install`, `ci`, `uninstall`, `link`, `dedupe`.
  * commands that manages the versioning, e.g., `version`.
  * commands that publish a package, e.g. `publish`.

In general, always use lerna commands if there is a equivalent available, see the lerna docs for information about the commands it supports.
