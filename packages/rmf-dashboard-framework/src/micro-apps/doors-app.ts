import { createMicroApp } from '.';

export default createMicroApp(
  'doors-table',
  'Doors',
  () => import('../components/doors/doors-table'),
  () => ({}),
);
