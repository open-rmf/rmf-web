import { lazy } from 'react';

import { MicroAppManifest } from '../components/micro-app';

export default {
  appId: 'tasks',
  displayName: 'Tasks',
  Component: lazy(() => import('../components/tasks/tasks-window')),
} satisfies MicroAppManifest;
