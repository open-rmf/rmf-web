[![Nightly](https://github.com/open-rmf/rmf-web/actions/workflows/nightly.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/nightly.yml) [![rmf-dashboard-framework](https://github.com/open-rmf/rmf-web/actions/workflows/rmf-dashboard-framework.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/rmf-dashboard-framework.yml) [![api-server](https://github.com/open-rmf/rmf-web/workflows/api-server/badge.svg)](https://github.com/open-rmf/rmf-web/actions?query=workflow%3Aapi-server+branch%3Amain) [![ros-translator](https://github.com/open-rmf/rmf-web/actions/workflows/ros-translator.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/ros-translator.yml) [![api-client](https://github.com/open-rmf/rmf-web/actions/workflows/api-client.yml/badge.svg)](https://github.com/open-rmf/rmf-web/actions/workflows/api-client.yml) [![codecov](https://codecov.io/gh/open-rmf/rmf-web/branch/main/graph/badge.svg)](https://codecov.io/gh/open-rmf/rmf-web)

# RMF Web

![](https://github.com/open-rmf/rmf-web/blob/media/dashboard1.0-office-world.gif)

Open-RMF Web is a collection of packages that provide a web-based interface for users to visualize and control all aspects of Open-RMF deployments.

- [Quick start with docker](#quick-start-with-docker)
- [Getting started from source](#getting-started-from-source)
- [API server](packages/api-server)
- [API client](packages/api-client)
- [RMF Dashboard Framework](packages/rmf-dashboard-framework)
- [Configuration](#configuration)
- [Contribution guide](#contribution-guide)
- [Roadmap](https://github.com/orgs/open-rmf/projects/10)

# Quick start with docker

These docker images are useful when trying out [`rmf_demos`](https://github.com/open-rmf/rmf_demos).

Start the demo dashboard with host network access, `RMF_SERVER_URL` and `TRAJECTORY_SERVER_URL` configured accordingly. The demo dashboard will then accessible on `localhost:3000` by default.

```bash
docker run \
  --network host -it --rm \
  -e RMF_SERVER_URL=http://localhost:8000 \
  -e TRAJECTORY_SERVER_URL=ws://localhost:8006 \
  ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly
```

> **Note**
> The demo dashboard assumes that the api server is hosted locally on the default ports.

Start the API server with host network access, and set up the correct `ROS_DOMAIN_ID` and ROS 2 RMW implementation (The images currently have `rmw_fastrtps_cpp` and `rmw_cyclonedds_cpp` installed and supported) that will be used in the rest of the Open-RMF system. The API server will use the default port at `localhost:8000`.

```bash
docker run \
  --network host -it --rm \
  -e ROS_DOMAIN_ID=<ROS_DOMAIN_ID> \
  -e RMW_IMPLEMENTATION=<RMW_IMPLEMENTATION> \
  ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly

# Use the appropriate tag for different ROS 2 distributions
```

> **Note**
> Users can also [configure the API server](packages/api-server/README.md/#configuration) using a mounted configuration file and setting the environment variable `RMF_API_SERVER_CONFIG`. In the default scenario, the API server will use an internal non-persistent database.

# Getting started from source

### Prerequisites

We currently support [Ubuntu 24.04](https://releases.ubuntu.com/noble/), [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html) and the latest Open-RMF release. Other distributions may work as well, but is not guaranteed.

Install pnpm and nodejs

```bash
curl -fsSL https://get.pnpm.io/install.sh | bash -
pnpm env use --global lts
```

For Debian/Ubuntu systems,

```bash
sudo apt install python3-pip python3-venv
```

### Installing Open-RMF

Refer to the following documentation for either building from source or installing released binaries:

- [rmf](https://github.com/open-rmf/rmf)

> **Note**
> Simulation demos are not part of the released binaries, and therefore a built workspace with at least the [demos repository](https://github.com/open-rmf/rmf_demos) would be required for trying out the web dashboard with simulation.

### Install dependencies

Run

```bash
pnpm install
```

You may also install dependencies for only a subset of the packages

```bash
pnpm install -w --filter <package>...
```

### Launching for development

Source Open-RMF and launch the API server,

```bash
# For binary installation
source /opt/ros/jazzy/setup.bash

# For source build
source /path/to/workspace/install/setup.bash

# Start the API server, this will need to be restarted for any changes to be reflected
cd packages/api-server
pnpm start
```

Start the dashboard in dev, this monitors for changes in the dashboard package and performs rebuilds. A browser refresh is sometimes required after automated builds to reflect new changes.

```bash
cd packages/rmf-dashboard-framework
pnpm start:example examples/demo
```

Once completed, the demo dashboard can be viewed at [localhost:5173](http://localhost:5173).

If presented with a login screen, use `user=admin password=admin`.

Ensure that the fleet adapters in the Open-RMF deployment is configured to use the endpoints of the API server. By default it is `http://localhost:8000/_internal`. Launching a simulation from [`rmf_demos_gz`](https://github.com/open-rmf/rmf_demos) for example, the command would be,

```bash
ros2 launch rmf_demos_gz office.launch.xml server_uri:="http://localhost:8000/_internal"
```

# Contribution guide

- For general contribution guidelines, see [CONTRIBUTING](CONTRIBUTING.md).
- Follow [typescript guidelines](https://basarat.gitbook.io/typescript/styleguide).
- When introducing a new feature in [`rmf-dashboard-framework`](packages/rmf-dashboard-framework), write tests as well as [e2e](packages/dashboard-e2e) test whenever possible.
- When introducing API changes with [`api-server`](packages/api-server),
  - If the new changes are to be used externally (outside of the web packages, with other Open-RMF packages for example), make changes to [`rmf_api_msgs`](https://github.com/open-rmf/rmf_api_msgs), before generating the required models using [this script](packages/api-server/generate-models.sh) with modified commit hashes.
  - Don't forget to update the API client with the newly added changes with [these instructions](packages/api-client/README.md/#generating-rest-api-client).
- Check out the latest API definitions [here](https://open-rmf.github.io/rmf-web/), or visit `/docs` relative to your running server's url, e.g. `http://localhost:8000/docs`.
- Develop the frontend without launching any Open-RMF components using [storybook](packages/dashboard/README.md/#storybook).
- For integration with new devices/infrastructure, check out [Robot Interaction Objects (RIO)](<https://github.com/open-rmf/rmf-web/wiki/Robot-Interaction-Objects-(RIO)>).
- Update documentation alongside development, and update the [`ros2multirobotbook`](https://osrf.github.io/ros2multirobotbook) where necessary.

# Configuration

- See the [api-server](packages/api-server/README.md#configuration) docs for API server run-time configurations.
- [rmf-dashboard-framework](packages/api-server/README.md) allows you to easily build a dashboard.

# Troubleshooting

- If a feature is missing or is not working, it could be only available in an Open-RMF source build, and not in the binaries. Try building Open-RMF from source and source that new workspace before launching the API server. `rmf-web` may use in-development features of Open-RMF.

- Creating tasks from the web dashboard when running a simulated Open-RMF deployment will require the task start time suit simulation time, which starts from unix millis 0. Try creating the same task with a start date of before the year of 1970.

- If floorplans for map levels are not loading, please check and verify that walls have been added to the levels in `.building.yaml` using `traffic-editor` or `rmf_site`. The dashboard uses the bounding box encompassing all wall vertices to create scene boundary for rendering, therefore if no wall vertices are present, the scene boundary becomes invalid and the floor fails to render.

- Check if the issue has already been [reported or fixed](https://github.com/open-rmf/rmf-web/issues).
