import { createMicroApp } from '../components';

export default createMicroApp(
  'lifts-table',
  'Lifts',
  () => import('../components/lifts/lifts-table'),
  () => ({}),
);
