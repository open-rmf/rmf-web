import * as RmfModels from 'rmf-models';
import { LeafletContext } from 'react-leaflet';

interface Location {
  x: number;
  y: number;
  yaw: number;
  level_name: string;
}

export interface RawDispenserResource {
  icons: Record<string, string>;
  location: Location;
}

export interface DispenserResource extends RawDispenserResource {
  guid: string;
}

export function dispenserModeToString(mode: number): string {
  switch (mode) {
    case RmfModels.DispenserState.IDLE:
      return 'IDLE';
    case RmfModels.DispenserState.BUSY:
      return 'ONLINE';
    case RmfModels.DispenserState.OFFLINE:
      return 'OFFLINE';
    default:
      return 'N/A';
  }
}

export function onWorkcellClick(workcell: DispenserResource, leafletMap?: LeafletContext) {
  leafletMap && leafletMap.map?.setView([workcell.location.y, workcell.location.x], 5.5);
}
