import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import './app.css';

import { appConfig } from './app-config';
import { RmfDashboard, StaticWorkspace, WorkspaceState } from './components';
import doorsApp from './micro-apps/doors-app';
import liftsApp from './micro-apps/lifts-app';
import createMapApp from './micro-apps/map-app';
import robotMutexGroupsApp from './micro-apps/robot-mutex-groups-app';
import robotsApp from './micro-apps/robots-app';
import tasksApp from './micro-apps/tasks-app';
import StubAuthenticator from './services/stub-authenticator';

const mapApp = createMapApp({
  attributionPrefix: appConfig.attributionPrefix,
  defaultMapLevel: appConfig.defaultMapLevel,
  defaultRobotZoom: appConfig.defaultRobotZoom,
  defaultZoom: appConfig.defaultZoom,
});

const homeWorkspace: WorkspaceState = {
  windows: {
    map: {
      layout: { x: 0, y: 0, w: 12, h: 6 },
      component: mapApp.Component,
    },
  },
};

const robotsWorkspace: WorkspaceState = {
  windows: {
    robots: {
      layout: { x: 0, y: 0, w: 7, h: 4 },
      component: robotsApp.Component,
    },
    map: { layout: { x: 8, y: 0, w: 5, h: 8 }, component: mapApp.Component },
    doors: { layout: { x: 0, y: 0, w: 7, h: 4 }, component: doorsApp.Component },
    lifts: { layout: { x: 0, y: 0, w: 7, h: 4 }, component: liftsApp.Component },
    mutexGroups: { layout: { x: 8, y: 0, w: 5, h: 4 }, component: robotMutexGroupsApp.Component },
  },
};

const tasksWorkspace: WorkspaceState = {
  windows: {
    tasks: { layout: { x: 0, y: 0, w: 7, h: 8 }, component: tasksApp.Component },
    map: { layout: { x: 8, y: 0, w: 5, h: 8 }, component: mapApp.Component },
  },
};

export default function App() {
  return (
    <RmfDashboard
      apiServerUrl="http://localhost:8000"
      trajectoryServerUrl="http://localhost:8006"
      authenticator={new StubAuthenticator()}
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
          element: <StaticWorkspace initialState={homeWorkspace} />,
        },
        {
          name: 'Robots',
          route: 'robots',
          element: <StaticWorkspace initialState={robotsWorkspace} />,
        },
        {
          name: 'Tasks',
          route: 'tasks',
          element: <StaticWorkspace initialState={tasksWorkspace} />,
        },
      ]}
    />
  );
}
