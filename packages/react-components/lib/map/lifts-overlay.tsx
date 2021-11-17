import React from 'react';
import * as RmfModels from 'rmf-models';
import { almostShallowEqual, fromRmfCoords, fromRmfYaw, radiansToDegrees } from '../utils';
import { DoorMarker as DoorMarker_ } from './door-marker';
import { useAutoScale } from './hooks';
import { LiftMarker as LiftMarker_, LiftMarkerProps, useLiftMarkerStyles } from './lift-marker';
import { SVGOverlay, SVGOverlayProps } from 'react-leaflet';
import { viewBoxFromLeafletBounds } from './utils';
import { withLabel } from './with-label';

function toDoorMode(liftState: RmfModels.LiftState): RmfModels.DoorMode {
  // LiftState uses its own enum definition of door state/mode which is separated from DoorMode.
  // But their definitions are equal so we can skip conversion.
  return { value: liftState.door_state };
}

interface BoundedMarkerProps extends Omit<LiftMarkerProps, 'onClick'> {
  lift: RmfModels.Lift;
  onClick?: (ev: React.MouseEvent, lift: string) => void;
}

/**
 * Bind a marker to include the lift name in the click event.
 * This is needed to avoid re-rendering all markers when only one of them changes.
 */
function bindMarker(MarkerComponent: React.ComponentType<LiftMarkerProps>) {
  return ({ lift, onClick, ...otherProps }: BoundedMarkerProps) => {
    const handleClick = React.useCallback((ev) => onClick && onClick(ev, lift.name), [
      onClick,
      lift.name,
    ]);
    return <MarkerComponent onClick={onClick && handleClick} {...otherProps} />;
  };
}

const DoorMarker = React.memo(DoorMarker_);
const LiftMarker = React.memo(withLabel(bindMarker(LiftMarker_)), (prev, next) =>
  almostShallowEqual(prev, next, ['style']),
);

export const getLiftModeVariant = (
  currentLevel: string,
  liftStateMode?: number,
  liftStateFloor?: string,
): keyof ReturnType<typeof useLiftMarkerStyles> | undefined => {
  if (!liftStateMode && !liftStateFloor) return 'unknown';
  if (liftStateMode === RmfModels.LiftState.MODE_FIRE) return 'fire';
  if (liftStateMode === RmfModels.LiftState.MODE_EMERGENCY) return 'emergency';
  if (liftStateMode === RmfModels.LiftState.MODE_OFFLINE) return 'offLine';
  if (liftStateFloor === currentLevel) {
    if (liftStateMode === RmfModels.LiftState.MODE_HUMAN) return 'human';
    if (liftStateMode === RmfModels.LiftState.MODE_AGV) return 'onCurrentLevel';
  } else {
    if (liftStateMode === RmfModels.LiftState.MODE_HUMAN) return 'moving';
    if (liftStateMode === RmfModels.LiftState.MODE_AGV) return 'moving';
  }
  if (liftStateMode === RmfModels.LiftState.MODE_UNKNOWN) return 'unknown';

  return 'unknown';
};

export interface LiftsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  currentLevel: string;
  lifts: RmfModels.Lift[];
  liftStates?: Record<string, RmfModels.LiftState>;
  hideLabels?: boolean;
  onLiftClick?: (ev: React.MouseEvent, lift: string) => void;
}

export const LiftsOverlay = ({
  lifts,
  liftStates = {},
  hideLabels = false,
  onLiftClick,
  currentLevel,
  ...otherProps
}: LiftsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);
  const scale = useAutoScale(40);

  return (
    <SVGOverlay attributes={{ viewBox: viewBox }} {...otherProps}>
      {lifts.map((lift) => {
        const state = liftStates && liftStates[lift.name];
        const pos = fromRmfCoords([lift.ref_x, lift.ref_y]);
        return (
          <g key={lift.name}>
            <LiftMarker
              lift={lift}
              onClick={onLiftClick}
              cx={pos[0]}
              cy={pos[1]}
              width={lift.width}
              height={lift.depth}
              yaw={radiansToDegrees(fromRmfYaw(lift.ref_yaw))}
              liftState={state}
              variant={getLiftModeVariant(
                currentLevel,
                liftStates[lift.name]?.current_mode,
                liftStates[lift.name]?.current_floor,
              )}
              style={{ transform: `scale(${scale})`, transformOrigin: `${pos[0]}px ${pos[1]}px` }}
              aria-label={lift.name}
              labelText={lift.name}
              labelSourceX={pos[0]}
              labelSourceY={pos[1]}
              labelSourceRadius={Math.min(lift.width / 2, lift.depth / 2)}
              labelArrowLength={Math.max((lift.width / 3) * scale, (lift.depth / 3) * scale)}
              hideLabel={hideLabels}
            />
            {lift.doors.map((door, idx) => {
              const [x1, y1] = fromRmfCoords([door.v1_x, door.v1_y]);
              const [x2, y2] = fromRmfCoords([door.v2_x, door.v2_y]);
              return (
                <DoorMarker
                  key={idx}
                  x1={x1}
                  y1={y1}
                  x2={x2}
                  y2={y2}
                  doorType={door.door_type}
                  doorMode={state && toDoorMode(state).value}
                />
              );
            })}
          </g>
        );
      })}
    </SVGOverlay>
  );
};

export default LiftsOverlay;
