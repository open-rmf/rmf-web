# rmf-dashboard-framework

`rmf-dashboard-framework` is a library that makes it easy to build a web application that provides visualization and control over the RMF system.

## Getting Started

See the [Getting Started](./docs/getting-started.md) docs.

## MicroApps

MicroApps are react components that can be used to build a page. The dashboard comes with many different micro-apps, each serving a different purpose when interacting with an Open-RMF deployment. The dashboard uses events to pipe information between micro-apps, for example clicking onto a task row in the Task app, will center the map onto the robot that the task was assigned to.

Some of the built in MicroApps are:
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

It is also possible to create new MicroApps, see [Creating a MicroApp](./docs/micro-apps.md#creating-a-microapp) for some examples.

## Workspaces

A `Workspace` is a react component that render MicroApps based on a given layout, it makes it very easy to build a page in `rmf-dashboard-framework` by combining multiple MicroApps. Here is an example of a workspace

```tsx
<Workspace initialWindows={[
  { layout: { x: 0, y: 0, w: 7, h: 8 }, microApp: tasksApp },
  { layout: { x: 8, y: 0, w: 5, h: 8 }, microApp: mapApp },
]} />
```

The above workspace renders a "Tasks" and "Map" MicroApp with a fixed layout, but it supports a `designMode` that allows free customizable of the layout by the user at runtime

```tsx
<LocallyPersistentWorkspace
  defaultWindows={[]}
  allowDesignMode
  appRegistry={[
    mapApp,
    doorsApp,
    liftsApp,
    robotsApp,
    robotMutexGroupsApp,
    tasksApp,
  ]}
  storageKey="custom-workspace"
/>
```

## Examples

See [examples](./examples/)
