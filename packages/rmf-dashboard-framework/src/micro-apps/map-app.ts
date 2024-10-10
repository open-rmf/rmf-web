import type { MapProps } from '../components/map';
import { createMicroApp, MicroAppManifest } from '.';

export default function createMapApp(config: MapProps): MicroAppManifest {
  return createMicroApp(
    'map',
    'Map',
    () => import('../components/map'),
    () => config,
  );
}
