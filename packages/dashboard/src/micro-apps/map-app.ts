import type { MapProps } from '../components/map';
import { createMicroApp, MicroAppManifest } from '../components/micro-app';

export default function createMapApp(config: MapProps): MicroAppManifest {
  return createMicroApp(
    'map',
    'Map',
    () => import('../components/map'),
    () => config,
  );
}
