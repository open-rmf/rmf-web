import { createMicroApp } from '.';

export default createMicroApp(
  'lifts-table',
  'Lifts',
  () => import('../components/lifts/lifts-table'),
  () => ({}),
);
