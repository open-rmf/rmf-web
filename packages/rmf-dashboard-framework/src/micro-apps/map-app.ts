import { createMicroApp, MicroAppManifest } from '../components';
import type { MapProps } from '../components/map/map';

export default function createMapApp(config: MapProps): MicroAppManifest {
  return createMicroApp(
    'map',
    'Map',
    () => import('../components/map/map'),
    () => config,
  );
}
