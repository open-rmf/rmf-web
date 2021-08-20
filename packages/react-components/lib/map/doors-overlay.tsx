import React from 'react';
import * as RmfModels from 'rmf-models';
import { DoorMarker as DoorMarker_, DoorMarkerProps } from './door-marker';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';

const DoorMarker = React.memo(DoorMarker_);

export interface DoorsOverlayProps extends SVGOverlayProps {
  doors: RmfModels.Door[];
  doorStates?: Record<string, RmfModels.DoorState>;
  onDoorClick?(ev: React.MouseEvent, door: RmfModels.Door): void;
}

export const DoorsOverlay = ({
  doors,
  doorStates = {},
  onDoorClick,
  bounds,
  ...otherProps
}: DoorsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(bounds);

  const handleDoorClick = React.useCallback<Required<DoorMarkerProps>['onClick']>(
    (ev, door) => onDoorClick && onDoorClick(ev, door),
    [onDoorClick],
  );

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {doors.map((door) => (
          <DoorMarker
            key={door.name}
            onClick={handleDoorClick}
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
