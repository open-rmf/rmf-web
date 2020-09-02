[![codecov](https://codecov.io/gh/osrf/romi-dashboard/branch/wip/graph/badge.svg)](https://codecov.io/gh/osrf/romi-dashboard)

## RoMi-dashboard

Hello! `romi-dashboard` is a web application that provides overall visualization and control over the RoMi-H system.

## Setup

Install `npm` and `nodejs`,

```bash
sudo apt update && sudo apt install curl
curl -sL https://deb.nodesource.com/setup_13.x | sudo -E bash -
sudo apt install nodejs
```

Clone the repository and install the required packages,

```bash
cd ~/.
git clone ssh://git@github.com/osrf/romi-dashboard
cd romi-dashboard
npm install

```

(Optional) Import external resources.

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

Launch the web application using the commands below,

```bash
cd ~/romi-dashboard
npm start
```

or alternatively, if using a different soss or trajectory server, the follow env variables can be defined

- REACT_APP_SOSS_SERVER
- REACT_APP_SOSS_TOKEN
- REACT_APP_TRAJECTORY_SERVER

and launched with

```bash
npm run start:custom
```

At this point, the dashboard should have been launched on the default browser of the machine, under `localhost:3000`.

For development, the page will reload if edits are made, while any lint errors will also appear on the console.

## Building for production

**Note: Not fully supported yet as it is missing a way to obtain the soss token.**

The following environment variables need to be defined

- REACT_APP_SOSS_SERVER
- REACT_APP_TRAJECTORY_SERVER

build using the command

```bash
npm run build
```

You can then serve the `build` directory using the webserver of your choice.
