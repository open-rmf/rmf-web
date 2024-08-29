import { lazy } from 'react';

import { MicroAppManifest } from '.';

export default {
  appId: 'tasks',
  displayName: 'Tasks',
  Component: lazy(() => import('../components/tasks/tasks-window')),
} satisfies MicroAppManifest;
