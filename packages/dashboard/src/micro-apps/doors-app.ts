import { createMicroApp } from '../components/micro-app';

export default createMicroApp(
  'doors-table',
  'Doors',
  () => import('../components/doors-table'),
  () => ({}),
);
