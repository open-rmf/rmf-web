import { createMicroApp } from '../components/micro-app';

export default createMicroApp(
  'robot-mutex-groups-table',
  'Mutex Groups',
  () => import('../components/robots/robot-mutex-group-table'),
  () => ({}),
);
