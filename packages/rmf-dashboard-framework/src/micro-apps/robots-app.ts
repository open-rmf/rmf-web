import { createMicroApp } from '.';

export default createMicroApp(
  'robots-table',
  'Robots',
  () => import('../components/robots/robots-table'),
  () => ({}),
);
