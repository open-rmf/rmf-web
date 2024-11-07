import { createMicroApp } from '../components';

export default createMicroApp(
  'robots-table',
  'Robots',
  () => import('../components/robots/robots-table'),
  () => ({}),
);
