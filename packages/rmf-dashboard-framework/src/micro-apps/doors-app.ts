import { createMicroApp } from '../components';

export default createMicroApp(
  'doors-table',
  'Doors',
  () => import('../components/doors/doors-table'),
  () => ({}),
);
