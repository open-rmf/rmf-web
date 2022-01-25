import type { Door, DoorState } from 'api-client';
import React from 'react';
import { almostShallowEqual } from '../utils';
import { fromRmfCoords } from '../utils/geometry';
import { DoorMarker as DoorMarker_, DoorMarkerProps } from './door-marker';
import { useAutoScale } from './hooks';
import { SVGOverlay, SVGOverlayProps } from './svg-overlay';
import { getDoorCenter, viewBoxFromLeafletBounds } from './utils';
import { withLabel } from './with-label';

interface BoundedMarkerProps extends Omit<DoorMarkerProps, 'onClick'> {
  door: Door;
  onClick?: (ev: React.MouseEvent, door: string) => void;
}

/**
 * Bind a marker to include the door name in the click event.
 * This is needed to avoid re-rendering all markers when only one of them changes.
 */
function bindMarker(MarkerComponent: React.ComponentType<DoorMarkerProps>) {
  return ({ door, onClick, ...otherProps }: BoundedMarkerProps) => {
    const handleClick = React.useCallback(
      (ev) => onClick && onClick(ev, door.name),
      [onClick, door.name],
    );
    return <MarkerComponent onClick={onClick && handleClick} {...otherProps} />;
  };
}

const DoorMarker = React.memo(withLabel(bindMarker(DoorMarker_)), (prev, next) =>
  almostShallowEqual(prev, next, ['style']),
);

export interface DoorsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  doors: Door[];
  doorStates?: Record<string, DoorState>;
  hideLabels?: boolean;
  onDoorClick?: (ev: React.MouseEvent, door: string) => void;
}

export const DoorsOverlay = ({
  doors,
  doorStates = {},
  hideLabels = false,
  onDoorClick,
  ...otherProps
}: DoorsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);
  const scale = useAutoScale(40);

  return (
    <SVGOverlay viewBox={viewBox} {...otherProps}>
      {doors.map((door) => {
        const center = fromRmfCoords(getDoorCenter(door));
        const [x1, y1] = fromRmfCoords([door.v1_x, door.v1_y]);
        const [x2, y2] = fromRmfCoords([door.v2_x, door.v2_y]);
        return (
          <DoorMarker
            key={door.name}
            door={door}
            onClick={onDoorClick}
            x1={x1}
            y1={y1}
            x2={x2}
            y2={y2}
            doorType={door.door_type}
            doorMode={
              doorStates && doorStates[door.name] && doorStates[door.name].current_mode.value
            }
            aria-label={door.name}
            style={{
              transform: `scale(${scale})`,
              transformOrigin: `${center[0]}px ${center[1]}px`,
            }}
            labelText={door.name}
            labelSourceX={center[0]}
            labelSourceY={center[1]}
            labelSourceRadius={0}
            hideLabel={hideLabels}
          />
        );
      })}
    </SVGOverlay>
  );
};

export default DoorsOverlay;
