## Getting Started

Welcome to RMF Dashboard Framework! This tutorial will guide you through the initial setup and provide a basic example to help you start building your dashboard application.

### Setup

Prerequisites:

- pnpm
- Latest Nodejs LTS

Follow https://pnpm.io/installation to install pnpm, you can then install nodejs using pnpm

```bash
pnpm env use --global lts
```

Clone this repo and install deps

<!-- TODO(koonpeng): install from npmjs after package is published. -->

```bash
pnpm create vite -t react-ts basic-dashboard
cd basic-dashboard
git clone --depth 1 https://github.com/open-rmf/rmf-web
```

<!-- TODO(koonpeng): not needed after package is published -->

Modify the following to resolve `rmf-dashboard-framework` as if it is installed from npmjs.

package.json

```
"dependencies": {
  ...
  "rmf-dashboard-framework": "rmf-web/packages/rmf-dashboard-framework"
}
```

tsconfig.app.json

```
"paths": {
  "rmf-dashboard-framework/*": ["./node_modules/rmf-dashboard-framework/src/*"]
}
```

Finally install and build the deps

```bash
# install and build rmf-dashboard-framework
cd rmf-web
pnpm install --filter=rmf-dashboard-framework...
pnpm --filter=rmf-dashboard-framework^... build
# install basic-dashboard deps
cd ..
pnpm install
```

### Creating a Basic Dashboard

At this point, you should be able to follow the [demo dashboard example](../examples/demo/main.tsx). You may need to install additional deps if you want to use the same fonts as the demo.

```bash
pnpm add -w @fontsource/roboto
```

### Starting a Dev Server

Before starting the dev server, we need a rmf api server backend for the frontend to connect to. If you are targeting an existing deployment, change the `apiServerUrl` prop to connect to the correct url. If you do not have an existing server, you can start a local server with docker.

<!-- FIXME(koonpeng): ROS_DOMAIN_ID and RMW_IMPLEMENTATION will be confusing to people not familiar with ROS. Also we can't really set the RMW_IMPLEMENTATION if the image does not have it installed. -->

```bash
docker run \
  --network host -it --rm \
  -e ROS_DOMAIN_ID=<ROS_DOMAIN_ID> \
  -e RMW_IMPLEMENTATION=<RMW_IMPLEMENTATION> \
  ghcr.io/open-rmf/rmf-web/api-server:latest
```

By default the testing server will be listening on http://localhost:8000, ensure that the `apiServerUrl` prop is correct.

Now you can start the actual vite dev server

```bash
pnpm dev
```

Navigate to http://localhost:5173 on a browser and you should see the dashboard.

### Starting RMF Simulation

If you are testing locally, you may see that the map, doors, robots etc are all empty, this is because the api server is not receiving any data from RMF. Check out [`rmf_demos`](https://github.com/open-rmf/rmf_demos) if you want to test with a RMF simulation.

### Adding a New Tab

The `tabs` props in `RmfDashboard` lets you configure the tabs available in the dashboard, you can add an item to the array to add a new tab, for example this will create a new tab that contain a single "Doors" MicroApp.

```tsx
{
  name: 'My New Tab',
  route: 'mynewtab',
  element: <Workspace initialWindows={[
    {
      layout: { x: 0, y: 0, w: 12, h: 6 },
      microApp: doorsApp,
    },
  ]}
  />,
}
```

The `layout` property defines the position and size of a MicroApp, the screen is divided into 12 columns and an unlimited number of rows, so this MicroApp will be positioned at the top left and have a width spanning the view port. For more information on the `layout` property, see [react-grid-layout](https://github.com/react-grid-layout/react-grid-layout).
