import { lazy } from 'react';
import { Suspense } from 'react';

import { MicroAppManifest } from '../components';

const TasksWindow = lazy(() => import('../components/tasks/tasks-window'));

export default {
  appId: 'tasks',
  displayName: 'Tasks',
  Component: (props) => (
    <Suspense fallback={null}>
      <TasksWindow {...props} />
    </Suspense>
  ),
} satisfies MicroAppManifest;
