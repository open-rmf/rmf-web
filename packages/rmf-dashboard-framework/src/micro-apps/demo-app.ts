import { createMicroApp } from '../components';

export default createMicroApp(
  'demo',
  'Microapp',
  () => import('../components/demo/demo'),
  () => ({}),
);
