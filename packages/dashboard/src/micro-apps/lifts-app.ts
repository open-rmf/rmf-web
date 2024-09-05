import { createMicroApp } from '../components/micro-app';

export default createMicroApp(
  'lifts-table',
  'Lifts',
  () => import('../components/lifts-table'),
  () => ({}),
);
