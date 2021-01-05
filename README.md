[![Nightly](https://github.com/osrf/rmf-web/workflows/Nightly/badge.svg)](https://github.com/osrf/rmf-web/actions?query=workflow%3ANightly)
[![codecov](https://codecov.io/gh/osrf/rmf-web/branch/main/graph/badge.svg)](https://codecov.io/gh/osrf/rmf-web)

# Installation

On Ubuntu 20.04, we need to install docker, docker-compose, and nodejs v12. Refer to the following for instruction to install them.

* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker-compose](https://docs.docker.com/compose/install/)
* [nodejs](https://nodejs.org/en/download/package-manager/)
  * alternative: [nvm](https://github.com/nvm-sh/nvm)

# Running the Dashboard

Before running the commands, make sure that the latest commit of the follow repos is built and sourced (the dashboard currently does not maintain any version support guarantees)
* [rmf_core](https://github.com/osrf/rmf_core)
* [traffic_editor](https://github.com/osrf/traffic_editor)
* [rmf schedule visualizer](https://github.com/osrf/rmf_schedule_visualizer)
* [rmf_demos](https://github.com/osrf/rmf_demos)
```bash
git clone https://github.com/osrf/rmf-web
cd rmf-web
npm run bootstrap
cd packages/dashboard
npm start
```

# Further documentation

Please see the [README](packages/dashboard/README.md) in the dashboard package.
