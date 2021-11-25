import { DispenserState as RmfDispenserState } from 'rmf-models';
import { LeafletContextInterface } from 'react-leaflet';

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
    case RmfDispenserState.IDLE:
      return 'IDLE';
    case RmfDispenserState.BUSY:
      return 'ONLINE';
    case RmfDispenserState.OFFLINE:
      return 'OFFLINE';
    default:
      return 'N/A';
  }
}

export function onWorkcellClick(
  workcell: DispenserResource,
  leafletMap?: LeafletContextInterface,
): void {
  leafletMap &&
    leafletMap.map?.setView([workcell.location.y, workcell.location.x], 5.5, {
      animate: true,
    });
}
