# rmf-dashboard

`rmf-dashboard` is a web application that provides overall visualization and control over the RMF system.

# Setup

Follow the build instructions [here](../../README.md/#launching).

## Configuration

Before building the dashboard, you must configure it according to the deployment. All the configurations is on `app-config.json`, the default configuration is for running the api-server and dashboard locally. You may check [src/app-config.ts](src/app-config.ts) for more details on the config file schema.

### (Optional) Add external resources.

The `resources` option in the app config allows configuring various assets used by the dashboard. At the moment, it mainly allows configuring the header logo and robot icons, they should point to either the full url or absolute url, for example

```json
  "resources": {
    "default": {
      "fleets": {
        "tinyRobot": {
          "default": {
            "icon": "https://raw.githubusercontent.com/open-rmf/rmf_demos/rmf-web-dashboard-resources/rmf_demos_dashboard_resources/office/icons/tinyRobot.png",
            "scale": 0.00123
          }
        }
      },
      "logos": {
        "header": "/resources/defaultLogo.png"
      }
    }
  },
```

In the above config, the header logo will be on absolute url `/resources/defaultLogo.png`, if the resources are hosted on a different domain (e.g. you are using a cdn), then it should include the full url, e.g. `https://storage.googleapis.com/[BUCKET_NAME]/[OBJECT_NAME]`.

For testing, it is convenient to put the resources in the `public` folder and they will be served by the vite dev server.

For dashboard resources for `rmf_demos` simulation worlds, check out the [rmf-web-dashboard-resources](https://github.com/open-rmf/rmf_demos/tree/rmf-web-dashboard-resources) branch of `rmf_demos`.

### Micro-apps

The dashboard comes with many different micro-apps, each serving a different purpose when interacting with an Open-RMF deployment. The dashboard uses events to pipe information between micro-apps, for example clicking onto a task row in the Task app, will center the map onto the robot that the task was assigned to.

Here are the available apps,
* Robots
* Map
* Doors
* Lifts
* Mutex Groups
* Tasks
* Beacons
* Robot Info
* Task Details
* Task Logs

### Workspace (tab) layouts

Each workspace (tab) allows users to define how the layout of micro-apps should be. By modifying the `WorkspaceState` of the workspace, micro-apps can be resized, moved, added or removed.

Here is an example workspace state,

```typescript
export const robotsWorkspace: WorkspaceState = {
  layout: [
    { i: 'robots', x: 0, y: 0, w: 7, h: 3 },
    { i: 'map', x: 8, y: 0, w: 5, h: 9 },
    { i: 'doors', x: 0, y: 0, w: 7, h: 3 },
    { i: 'lifts', x: 0, y: 0, w: 7, h: 3 },
    { i: 'beacons', x: 0, y: 0, w: 7, h: 3 },
    { i: 'mutexGroups', x: 8, y: 0, w: 5, h: 3 },
  ],
  windows: [
    { key: 'robots', appName: 'Robots' },
    { key: 'map', appName: 'Map' },
    { key: 'doors', appName: 'Doors' },
    { key: 'lifts', appName: 'Lifts' },
    { key: 'beacons', appName: 'Beacons' },
    { key: 'mutexGroups', appName: 'Mutex Groups' },
  ],
};
```

### Custom tab(s)

With the dashboard configuration parameter `customTabs: true`, 2 custom tabs would be available on the App bar. These custom tabs allow users to create a custom layout of desired micro-apps, if a custom workflow is desired.

These custom layouts will be cached locally on the browser's machine, where it can be brought up again.

To edit a custom tab, click onto the wand icon on the right end of the App bar, and proceed to add, resize, move or remove micro-apps from the layout.

![](https://github.com/open-rmf/rmf-web/blob/media/custom-tabs.gif)
