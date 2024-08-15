import type { MapProps } from './map';
import { createMicroApp, MicroAppManifest } from './micro-app';

export function createMapApp(config: MapProps): MicroAppManifest {
  return createMicroApp(
    'map',
    'Map',
    () => import('./map'),
    () => config,
  );
}
