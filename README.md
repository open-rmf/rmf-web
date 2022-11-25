[![Nightly](https://github.com/open-rmf/rmf-web/actions/workflows/nightly.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/nightly.yml) [![Dashboard End-to-End](https://github.com/open-rmf/rmf-web/actions/workflows/dashboard-e2e.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/dashboard-e2e.yml) [![react-components](https://github.com/open-rmf/rmf-web/workflows/react-components/badge.svg)](https://github.com/open-rmf/rmf-web/actions?query=workflow%3Areact-components+branch%3Amain) [![dashboard](https://github.com/open-rmf/rmf-web/workflows/dashboard/badge.svg)](https://github.com/open-rmf/rmf-web/actions?query=workflow%3Adashboard+branch%3Amain) [![api-server](https://github.com/open-rmf/rmf-web/workflows/api-server/badge.svg)](https://github.com/open-rmf/rmf-web/actions?query=workflow%3Aapi-server+branch%3Amain) [![rmf-auth](https://github.com/open-rmf/rmf-web/actions/workflows/rmf-auth.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/rmf-auth.yml) [![ros-translator](https://github.com/open-rmf/rmf-web/actions/workflows/ros-translator.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/ros-translator.yml) [![api-client](https://github.com/open-rmf/rmf-web/actions/workflows/api-client.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/api-client.yml) [![codecov](https://codecov.io/gh/open-rmf/rmf-web/branch/main/graph/badge.svg)](https://codecov.io/gh/open-rmf/rmf-web)

# Building the Dashboard

## Prerequisites

### Ubuntu 22.04

Install nodejs
```bash
sudo apt update && sudo apt install curl
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.2/install.sh | bash
nvm install 16
```

Install pnpm and nodejs
```bash
curl -fsSL https://get.pnpm.io/install.sh | bash -
pnpm env use --global 16
```

### Install rmf

Refer to the following documentation:

* [rmf](https://github.com/open-rmf/rmf)

### Others

Refer to the following documentations:

* [nodejs](https://nodejs.org/en/download/package-manager/) >= 16
* [rmf_demos](https://github.com/open-rmf/rmf_demos)

## Install dependencies

For Debian/Ubuntu systems, you may need to install `python3-venv` first.
```bash
sudo apt install python3-venv
```

Run
```bash
pnpm install
```

You may also install dependencies for only a subset of the packages
```bash
pnpm install -w --filter <package>...
```

### PostgreSQL
If you would like to use PostgreSQL, you will also need to install and set it up. The defaults are for PostgreSQL to be listening on 127.0.0.1:5432.

#### Docker
We can use Docker to quickly bring up a PostgreSQL instance.

Install docker: `https://docs.docker.com/engine/install/ubuntu/`
Start a a database instance: `docker run -it --rm --name rmf-postgres --network=host -e POSTGRES_PASSWORD=postgres -d postgres`

To stop the instance: `docker kill rmf-postgres`


#### Bare Metal
Alternatively, we can install PostgreSQL 'bare metal'.
```
apt install postgresql postgresql-contrib -y
# Set a default password
sudo -u postgres psql -c "ALTER USER postgres PASSWORD 'postgres';"

sudo systemctl restart postgresql
# interactive prompt
sudo -i -u postgres
```
To manually reset the database:
```
sudo -u postgres bash -c "dropdb postgres; createdb postgres"
```

## Launching

Before running the commands, make sure that rmf is sourced.
```bash
cd packages/dashboard
pnpm start
```
When presented with a login screen, use `user=admin password=admin`.

This launches a development server with the office world from `rmf_demos`. The server is useful for development but is obviously not useful for actual usage.

## Configuration

See the [rmf-dashboard](packages/dashboard/README.md#configuration) docs.

## Troubleshooting
First thing to try is to build rmf from source, in order to speed up development, `rmf-web` may use in-development features of rmf. That means that the binary releases may not have the features required, sometimes the features `rmf-web` uses may be so new that not even the rolling releases has it.

Refer to [rmf_demos](https://github.com/open-rmf/rmf_demos) for instructions to build rmf. You should end up with a colcon workspace with all of rmf packages, remember to source the workspace before running any of the commands.
