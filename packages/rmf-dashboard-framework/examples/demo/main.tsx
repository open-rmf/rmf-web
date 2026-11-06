import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';

import ReactDOM from 'react-dom/client';
import {
  InitialWindow,
  LocallyPersistentWorkspace,
  MicroAppManifest,
  RmfDashboard,
  Workspace,
} from 'rmf-dashboard-framework/components';
import {
  createMapApp,
  doorsApp,
  liftsApp,
  robotMutexGroupsApp,
  robotsApp,
  tasksApp,
} from 'rmf-dashboard-framework/micro-apps';
import { StubAuthenticator } from 'rmf-dashboard-framework/services';

declare global {
  interface Window {
    RMF_SERVER_URL?: string;
    TRAJECTORY_SERVER_URL?: string;
  }
}

// The docker image injects these at container start (see index.html), otherwise
// the placeholders are left as-is and we fall back to the default local ports.
function serverUrl(injected: string | undefined, fallback: string): string {
  return injected && !injected.startsWith('__') ? injected : fallback;
}
const apiServerUrl = serverUrl(window.RMF_SERVER_URL, 'http://localhost:8000');
const trajectoryServerUrl = serverUrl(window.TRAJECTORY_SERVER_URL, 'http://localhost:8006');

const mapApp = createMapApp({
  attributionPrefix: 'Open-RMF',
  defaultMapLevel: 'L1',
  defaultRobotZoom: 20,
  defaultZoom: 6,
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
      apiServerUrl={apiServerUrl}
      trajectoryServerUrl={trajectoryServerUrl}
      authenticator={new StubAuthenticator()}
      helpLink="https://osrf.github.io/ros2multirobotbook/rmf-core.html"
      reportIssueLink="https://github.com/open-rmf/rmf-web/issues"
      resources={{ fleets: {}, logos: { header: '/resources/defaultLogo.png' } }}
      tasks={{
        allowedTasks: [
          { taskDefinitionId: 'patrol' },
          { taskDefinitionId: 'delivery' },
          { taskDefinitionId: 'compose-clean' },
          { taskDefinitionId: 'custom_compose' },
        ],
        pickupZones: [],
        cartIds: [],
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

const root = ReactDOM.createRoot(document.getElementById('root') as HTMLElement);
root.render(<App />);
