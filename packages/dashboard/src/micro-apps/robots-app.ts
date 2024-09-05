import { createMicroApp } from '../components/micro-app';

export default createMicroApp(
  'robots-table',
  'Robots',
  () => import('../components/robots/robots-table'),
  () => ({}),
);
