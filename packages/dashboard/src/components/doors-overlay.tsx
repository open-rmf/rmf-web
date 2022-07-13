import { Door, DoorState } from 'api-client';
import React from 'react';
import {
  DoorMarker as BaseDoorMarker,
  DoorMarkerProps as BaseDoorMarkerProps,
  fromRmfCoords,
  getDoorCenter,
  SVGOverlay,
  SVGOverlayProps,
  useAutoScale,
  viewBoxFromLeafletBounds,
  withLabel,
} from 'react-components';
import { RmfAppContext } from './rmf-app';

interface DoorMarkerProps extends Omit<BaseDoorMarkerProps, 'doorMode'> {
  door: Door;
}

const DoorMarker = withLabel(({ door, ...otherProps }: DoorMarkerProps) => {
  const rmf = React.useContext(RmfAppContext);
  const [doorState, setDoorState] = React.useState<DoorState | null>(null);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.getDoorStateObs(door.name).subscribe(setDoorState);
    return () => sub.unsubscribe();
  }, [rmf, door]);

  return (
    <BaseDoorMarker
      doorMode={doorState ? doorState.current_mode.value : undefined}
      {...otherProps}
    />
  );
});

export interface DoorsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  doors: Door[];
  hideLabels?: boolean;
  onDoorClick?: (ev: React.MouseEvent, door: Door) => void;
}

export const DoorsOverlay = React.memo(
  ({ doors, hideLabels = false, onDoorClick, ...otherProps }: DoorsOverlayProps): JSX.Element => {
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
              onClick={(ev) => onDoorClick && onDoorClick(ev, door)}
              x1={x1}
              y1={y1}
              x2={x2}
              y2={y2}
              doorType={door.door_type}
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
  },
);
