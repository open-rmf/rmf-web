import React from 'react';
import * as RmfModels from 'rmf-models';
import { DoorMarker as DoorMarker_, DoorMarkerProps } from './door-marker';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';

interface BoundedMarkerProps extends Omit<DoorMarkerProps, 'onClick'> {
  onClick?: (ev: React.MouseEvent, door: string) => void;
}

/**
 * Bind a marker to include the door name in the click event.
 * This is needed to avoid re-rendering all markers when only one of them changes.
 */
function bindMarker(MarkerComponent: React.ComponentType<DoorMarkerProps>) {
  return ({ onClick, ...otherProps }: BoundedMarkerProps) => {
    const handleClick = React.useCallback((ev) => onClick && onClick(ev, otherProps.door.name), [
      onClick,
      otherProps.door.name,
    ]);
    return <MarkerComponent onClick={onClick && handleClick} {...otherProps} />;
  };
}

const DoorMarker = React.memo(bindMarker(DoorMarker_));

export interface DoorsOverlayProps extends SVGOverlayProps {
  doors: RmfModels.Door[];
  doorStates?: Record<string, RmfModels.DoorState>;
  onDoorClick?: (ev: React.MouseEvent, door: string) => void;
}

export const DoorsOverlay = ({
  doors,
  doorStates = {},
  onDoorClick,
  bounds,
  ...otherProps
}: DoorsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(bounds);

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {doors.map((door) => (
          <DoorMarker
            key={door.name}
            onClick={onDoorClick}
            door={door}
            doorMode={
              doorStates && doorStates[door.name] && doorStates[door.name].current_mode.value
            }
            aria-label={door.name}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
};

export default DoorsOverlay;
