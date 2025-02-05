import { createMicroApp } from '../components';

export default createMicroApp(
  'demo',
  'Demo',
  () => import('../components/demo/demo'),
  () => ({}),
);
