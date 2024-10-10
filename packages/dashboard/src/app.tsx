import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import './app.css';

import { appConfig, authenticator } from './app-config';
import { InitialWindow, LocallyPersistentWorkspace, RmfDashboard, Workspace } from './components';
import { MicroAppManifest } from './components/micro-app';
import doorsApp from './micro-apps/doors-app';
import liftsApp from './micro-apps/lifts-app';
import createMapApp from './micro-apps/map-app';
import robotMutexGroupsApp from './micro-apps/robot-mutex-groups-app';
import robotsApp from './micro-apps/robots-app';
import tasksApp from './micro-apps/tasks-app';

const mapApp = createMapApp({
  attributionPrefix: appConfig.attributionPrefix,
  defaultMapLevel: appConfig.defaultMapLevel,
  defaultRobotZoom: appConfig.defaultRobotZoom,
  defaultZoom: appConfig.defaultZoom,
});

const appRegistry: MicroAppManifest[] = [
  mapApp,
  doorsApp,
  liftsApp,
  robotsApp,
  robotMutexGroupsApp,
  tasksApp,
];

const homeWorkspace: InitialWindow[] = [
  {
    layout: { x: 0, y: 0, w: 12, h: 6 },
    microApp: mapApp,
  },
];

const robotsWorkspace: InitialWindow[] = [
  {
    layout: { x: 0, y: 0, w: 7, h: 4 },
    microApp: robotsApp,
  },
  { layout: { x: 8, y: 0, w: 5, h: 8 }, microApp: mapApp },
  { layout: { x: 0, y: 0, w: 7, h: 4 }, microApp: doorsApp },
  { layout: { x: 0, y: 0, w: 7, h: 4 }, microApp: liftsApp },
  { layout: { x: 8, y: 0, w: 5, h: 4 }, microApp: robotMutexGroupsApp },
];

const tasksWorkspace: InitialWindow[] = [
  { layout: { x: 0, y: 0, w: 7, h: 8 }, microApp: tasksApp },
  { layout: { x: 8, y: 0, w: 5, h: 8 }, microApp: mapApp },
];

export default function App() {
  return (
    <RmfDashboard
      apiServerUrl={appConfig.rmfServerUrl}
      trajectoryServerUrl={appConfig.trajectoryServerUrl}
      authenticator={authenticator}
      helpLink={appConfig.helpLink}
      reportIssueLink={appConfig.reportIssue}
      resources={appConfig.resources.default}
      tasks={{
        allowedTasks: appConfig.allowedTasks,
        pickupZones: appConfig.pickupZones,
        cartIds: appConfig.cartIds,
      }}
      tabs={[
        {
          name: 'Map',
          route: '',
          element: <Workspace initialWindows={homeWorkspace} />,
        },
        {
          name: 'Robots',
          route: 'robots',
          element: <Workspace initialWindows={robotsWorkspace} />,
        },
        {
          name: 'Tasks',
          route: 'tasks',
          element: <Workspace initialWindows={tasksWorkspace} />,
        },
        {
          name: 'Custom',
          route: 'custom',
          element: (
            <LocallyPersistentWorkspace
              defaultWindows={[]}
              allowDesignMode
              appRegistry={appRegistry}
              storageKey="custom-workspace"
            />
          ),
        },
      ]}
    />
  );
}
