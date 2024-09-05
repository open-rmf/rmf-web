import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';

import { createTheme } from '@mui/material';
import ReactDOM from 'react-dom/client';
import { LocallyPersistentWorkspace, RmfDashboard } from 'rmf-dashboard/components';
import { MicroAppManifest } from 'rmf-dashboard/components/micro-app';
import doorsApp from 'rmf-dashboard/micro-apps/doors-app';
import liftsApp from 'rmf-dashboard/micro-apps/lifts-app';
import createMapApp from 'rmf-dashboard/micro-apps/map-app';
import robotMutexGroupsApp from 'rmf-dashboard/micro-apps/robot-mutex-groups-app';
import robotsApp from 'rmf-dashboard/micro-apps/robots-app';
import tasksApp from 'rmf-dashboard/micro-apps/tasks-app';
import StubAuthenticator from 'rmf-dashboard/services/stub-authenticator';

/* eslint-disable @typescript-eslint/no-unused-vars,@typescript-eslint/ban-ts-comment */
// Polar Night
const nord0 = '#2e3440'; // @ts-ignore
const nord1 = '#3b4252'; // @ts-ignore
const nord2 = '#434c5e'; // @ts-ignore
const nord3 = '#4c566a'; // @ts-ignore

// Snow Storm
const nord4 = '#d8dee9'; // @ts-ignore
const nord5 = '#e5e9f0'; // @ts-ignore
const nord6 = '#eceff4'; // @ts-ignore

// Frost
const nord7 = '#8fbcbb'; // @ts-ignore
const nord8 = '#88c0d0'; // @ts-ignore
const nord9 = '#81a1c1'; // @ts-ignore
const nord10 = '#5e81ac'; // @ts-ignore

// Aurora
const nord11 = '#bf616a'; // @ts-ignore
const nord12 = '#d08770'; // @ts-ignore
const nord13 = '#ebcb8b'; // @ts-ignore
const nord14 = '#a3be8c'; // @ts-ignore
const nord15 = '#b48ead'; // @ts-ignore
/* eslint-enable @typescript-eslint/no-unused-vars,@typescript-eslint/ban-ts-comment */

const nordTheme = createTheme({
  palette: {
    mode: 'dark',
    primary: {
      main: nord8,
      contrastText: nord1,
    },
    secondary: {
      main: nord9,
    },
    text: {
      primary: nord4,
      secondary: nord6,
      disabled: nord5,
    },
    error: {
      main: nord11,
    },
    warning: {
      main: nord13,
    },
    success: {
      main: nord14,
    },
    background: { default: nord0, paper: nord1 },
  },
});

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

export default function App() {
  return (
    <RmfDashboard
      apiServerUrl="http://localhost:8000"
      trajectoryServerUrl="http://localhost:8006"
      authenticator={new StubAuthenticator()}
      helpLink="https://osrf.github.io/ros2multirobotbook/rmf-core.html"
      reportIssueLink="https://github.com/open-rmf/rmf-web/issues"
      themes={{ default: createTheme(), dark: nordTheme }}
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
          name: 'Example',
          route: '',
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
