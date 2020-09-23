![Nightly](https://github.com/osrf/romi-dashboard/workflows/Nightly/badge.svg)
[![codecov](https://codecov.io/gh/osrf/romi-dashboard/branch/master/graph/badge.svg)](https://codecov.io/gh/osrf/romi-dashboard)

## RoMi-dashboard

Hello! `romi-dashboard` is a web application that provides overall visualization and control over the RoMi system.

## Setup

Install `npm` and `nodejs`,

```bash
sudo apt update && sudo apt install curl
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt install nodejs
```

Clone the repository and install the required packages,

```bash
cd ~/.
git clone https://github.com/osrf/romi-dashboard
cd romi-dashboard
npm install
```

### (Optional) Import external resources.

```bash
npm run setup
```

When executing this command, a prompt will be displayed, and it will guide you through the process.

You have two options, import a specific folder with assets from a repository (like Github) or copy the assets folder from a local directory.

In the case of choosing to obtain the data from a repository, you must define: (1) The repository URL and (2) the folder within the repository. In case of choosing to obtain the data from a local directory, the absolute path must be defined.

This folder obtained must contain at its root a JSON file calling `main.json`, with the _RomiDashboard_ actors (robots, dispensers, etc.) and the path to their corresponding resources. E.g.:

```json
{
  "robots":
    "tinyRobot" <fleet>: {
      "icons": {
        "tinyRobot"<fleet>: "/robots/tinyRobot/tinyRobot.png",
        "model1"<model>: "/robots/tinyRobot/model1.png"
      }
    }
}
```

Obs: the algorithm will check the robot's icons in this order:

1. Icon exists for a model; if not exists
2. Icon exists for a fleet; if not exists
3. It will use a default icon.

Upon completion of the configuration, a file called .resources.json will be created automatically, which will save the configuration from where to obtain the resources.

In case you want to modify the source of your resources. You can rerun the command:

```bash
npm run setup
```

## Launching

### Before Launching

RoMi-dashboard connects to soss using https, if you are using the local or docker launch mode, you need to make sure that the browser does not reject the self-signed certed. The recommended way to do it is to tell the browser to skip the certificate verification for localhosts, if you are using chrome you can follow the instructions [here](https://community.spiceworks.com/how_to/154601-chrome-allow-localhost-site-even-without-https-certificate#:~:text=Step%201%3A%20Open%20Google%20Chrome,address%20bar%20in%20google%20chrome.&text=Set%20the%20option%20to%20enabled,to%20deal%20with%20https%20warnings.).

Another way is to add the ca found in `e2e/certs/ca.crt` to the trusted certificate authorities. **Doing this is NOT recommended because the CA private key is exposed publicly, this may open you up to MITM attacks by using the exposed key to forge fake certificate that your browser trusts.**

### Local Dev Server

_RoMi-dashboard_ gets its data source from the following rmf components:

- [rmf schedule visualizer](https://github.com/osrf/rmf_schedule_visualizer)
- [soss](https://github.com/osrf/soss)
- [rmf-soss-ros2](https://github.com/osrf/rmf-soss-ros2)
- [rmf_demos](https://github.com/osrf/rmf_demos)

Refer to the respective projects for instruction on how to setup them up.

You would also need [docker](https://docs.docker.com/engine/install/ubuntu/) and [docker-compose](https://docs.docker.com/compose/install/) for the authentication server.

The easiest way to launch a local server for development is with

```bash
npm start
```

This will start all the necessary backends, a browser window should open up pointing to the local dev server. When prompted with an user/password, use this:

```
user: admin
password: admin
```

### Docker Based Backend

If you have problem setting up rmf and soss, you can make use of the docker image used by the e2e tests to run the backend in docker.

First, download the docker images, the images are hosted on github packages so you will need a github account to access it, refer to [here](https://docs.github.com/en/packages/using-github-packages-with-your-projects-ecosystem/configuring-docker-for-use-with-github-packages) for instructions.

After you have the credentials set up, run this to download the images

```bash
npm run sync:docker
```

Then run this to start the dev services

```bash
npm run start:docker
```

This will start all the services in a docker container, the react web app will be hosted at `localhost:3000`.

The rmf image is built nightly, if you would like to test against the latest build, be sure to update the docker images regularly.

#### Building Docker Images Locally

If you would like, you can also build the images locally, doing so is simple with

```bash
# build rmf image
docker-compose -f <path-to-romi-dashboard>/docker/rmf/docker-compose.yml build --no-cache
# build e2e image
docker-compose -f <path-to-romi-dashboard>/docker/docker-compose.yml build --no-cache e2e
```

This will download and build all of rmf so it may take awhile.

### External Server

Alternatively, if you want to connect to an existing rmf deployment, set the following environment variables:

- _REACT_APP_SOSS_SERVER_: URL to the soss server.
- _REACT_APP_TRAJECTORY_SERVER_: URL to the trajectory server.
- _REACT_APP_AUTH_CONFIG_: A JSON object containing the following
  - _realm_: The keycloak realm
  - _clientId_: clientId
  - _url_: URL to the keycloak server

Then start the web server with

```bash
npm run start:react
```

### Mock Data

If you are just working on the frontend, you can launch _RoMi-dashboard_ with a set of mock data, this does not require any rmf deployments, simply run

```bash
npm run start:mock
```

To bring up a web server using mock data.

### Storybook

Another way to work on the frontend without rmf is with [storybook](https://storybook.js.org/), simply run

```bash
npm run storybook
```

This is ideal if you are working on individual isolated components.

## Building for production

Firstly, set up environment variables according to [this](#External-Server), you may also want to install a resource pack as described [here](#Optional-Import-external-resources.). Then build a production version of _RoMi-dashboard_ with

```bash
npm run build
```

You can then serve the `build` directory using the webserver of your choice.
