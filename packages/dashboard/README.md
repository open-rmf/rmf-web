# rmf-dashboard

`rmf-dashboard` is a web application that provides overall visualization and control over the RMF system.

# Setup

Follow the build instructions [here](../../README.md/#launching).

## Configuration

### (Optional) Import external resources.

This allows for build-time configurations of the dashboard.

This process can be done manually, or to use the helpful command,

```bash
cd rmf-web/packages/dashboard
pnpm run setup
```

When executing this command, a prompt will be displayed, and it will guide you through the process.

You have two options, import a specific folder with assets from a repository (like Github) or copy the assets folder from a local directory.

In the case of choosing to obtain the data from a repository, you must define: (1) The repository URL and (2) the folder within the repository. In case of choosing to obtain the data from a local directory, the absolute path must be defined.

This folder obtained must contain at its root a JSON file calling `main.json`, with
* `robots`, where each fleet of robots can have different
  * `icons`, path to the icon image
  * `scale`, the scale of the icon in meters/pixel, for the size to be rendered accurately in the map
* `dispensers`, where each dispenser can define its own
  * `icons`, path to the icon image
  * `location`, x, y, yaw, and level
* `logos`, where named logos can be defined, most prominently `headerLogo` for the dashboard, default to Open-RMF logo
* `helpLink`, changes the link of the help button on the `appbar`, defaults to the [`ros2multirobotbook`](https://osrf.github.io/ros2multirobotbook/rmf-core.html)
* `reportIssue`, changes the link of the issue button on the `appbar`, defaults to [rmf-web issues](https://github.com/open-rmf/rmf-web/issues)
* `defaultZoom`, the default zoom level of the map whenever the page is refreshed and the map is rendered, default value as `5` (the higher the value, the larger the entities appear in the map view)
* `defaultRobotZoom`, the zoom level when the map's view is centered on a robot, whenever a task row or robot row is clicked, default value as `40` (the higher the value, the larger the robot icon appears in the map view)
* `attributionPrefix`, the attribution text on the right bottom corner of the map, default as `OSRF`
* `loggedInDisplayLevel`, the name of the map level to be displayed whenever a user logs in or the page is refreshed, defaults to the first level in the building map

Custom fields can also be easily added and used in the dashboard, some examples,
* `pickupZones`, this is a custom field implemented to allow a custom list of pickup zones to be used in a task form dropdown list
* `cartIds`, this is a custom field implemented to allow a custom list of cart IDs to be used in a task form dropdown list

An example directory structure can be found in `rmf_demos`, under the branch [`rmf-web-dashboard-resources`](https://github.com/open-rmf/rmf_demos/tree/rmf-web-dashboard-resources/rmf_demos_dashboard_resources/office)

Here is another example `main.json`,

```json
{
  "robots": {
    "tinyRobot": {
      "icons": {
        "tinyRobot": "/robots/tinyRobot/tinyRobot.png",
        "model1": "/robots/tinyRobot/model1.png"
      },
      "scale": 0.004
    }
  },
  "dispensers": {
    "coke_dispenser": {
      "icons":{
        "coke_dispenser": "/icons/fridge.png"
      },
      "location": {
        "x":16.85,
        "y":-4.8,
        "yaw":0,
        "level_name":"L1"
      }
    },
  },
  "logos":{
    "headerLogo":{
      "icons":{
        "headerLogo":"/icons/headerLogo.png"
      }
    }
  },
  "helpLink": "insert-help-link-here",
  "reportIssue": "insert-report-issue-link-here",
  "defaultZoom": 10,
  "defaultRobotZoom": 50,
  "attributionPrefix": "insert-attribution-prefix-here",
  "loggedInDisplayLevel": "default_display_level_name_in_building_map"
}
```

Note, the algorithm will check the robot's icons in this order:

1. Icon exists for a model; if not exists
2. Icon exists for a fleet; if not exists
3. It will use a default icon.

Upon completion of the configuration, a file called .resources.json will be created automatically, which will save the configuration from where to obtain the resources.

In case you want to modify the source of your resources. You can rerun the command:

```bash
pnpm run setup
```

### Environment Variables

The default launch script with `pnpm start` launches only the backend servers without any simulation instances from `rmf_demos`. For local development, the launch script `pnpm run start:sim` launches a headless simulation instance on top of all the backend servers.

In production, the dashboard would need to be configured so that it knows where to find the various services.

Here are the environment variables that you can set before you build the dashboard:

* _PUBLIC_URL_: Url that the app is hosted. Defaults to '/'.
* _REACT_APP_TRAJECTORY_SERVER_: **Required** URL to the trajectory server.
* _REACT_APP_RMF_SERVER_: **Required** URL to the RMF api server.
* _REACT_APP_AUTH_PROVIDER_: A string indicating the auth provider to use, defaults to the stub authenticator.

Supported auth providers are
| provider | config |
|----------|--------|
| stub | N/A |
| keycloak | _REACT_APP_KEYCLOAK_CONFIG_: a json string with the keys _realm_, _clientId_ and _url_ |

## Storybook

**Many of the components are reused from `react-components`, you may want to run storybook on `react-components` instead.**

Another way to work on the frontend without rmf is with [storybook](https://storybook.js.org/), simply run

```bash
npm run storybook
```

This is ideal if you are working on individual isolated components.
