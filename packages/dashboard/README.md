## RoMi-dashboard

Hello! `romi-dashboard` is a web application that provides overall visualization and control over the RoMi system.

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

### Local Dev Server

You must have an installation of RMF sourced, you must have also bootstrap all of the packages, if you follow the instructions from the [main README](../../README.md), you should already have things ready.

To start the dev server, run

```bash
npm start
```

This will start all the necessary backends, a browser window should open up pointing to the local dev server. When prompted with an user/password, use this:

```
user: admin
password: admin
```

### Connecting to External Server

Alternatively, if you want to connect to an existing rmf deployment, or if you want to manage one or more backends manually, set the following environment variables:

* _PUBLIC_URL_: Url that the app is hosted. Defaults to '/'.
* _REACT_APP_TRAJECTORY_SERVER_: **Required** URL to the trajectory server.
* _REACT_APP_ROS2_BRIDGE_SERVER_: **Required** URL to the ros2 bridge server.
* _REACT_APP_AUTH_PROVIDER_: A string indicating the auth provider to use, defaults to the stub authenticator.

Supported auth providers are
| provider | config |
|----------|--------|
| stub | N/A |
| keycloak | _REACT_APP_KEYCLOAK_CONFIG_: a json string with the keys _realm_, _clientId_ and _url_ |

### Storybook

**Many of the components are reused from `react-components`, you may want to run storybook on `react-components` instead.**

If you are working on the frontend, it is recommended to use [storybook](https://storybook.js.org/) to test your components in isolation before integrating them into the dashboard for e2e tests.

```bash
npm run storybook
```

## Building for production

Firstly, set up environment variables according to [this](#External-Server), you may also want to install a resource pack as described [here](#Optional-Import-external-resources.). Then build a production version of _RoMi-dashboard_ with

```bash
npm run build
```

You can then serve the `build` directory using the webserver of your choice.
