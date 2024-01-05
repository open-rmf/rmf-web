import { Door, DoorMode, Lift, LiftState } from 'api-client';
import React from 'react';
import {
  DoorMarker,
  fromRmfCoords,
  fromRmfYaw,
  LiftMarker as BaseLiftMarker,
  liftMarkerClasses,
  LiftMarkerProps as BaseLiftMarkerProps,
  radiansToDegrees,
  SVGOverlay,
  SVGOverlayProps,
  useAutoScale,
  viewBoxFromLeafletBounds,
  withLabel,
} from 'react-components';
import { LiftState as RmfLiftState } from 'rmf-models';
import { RmfAppContext } from './rmf-app';

function toDoorMode(liftState: LiftState): DoorMode {
  // LiftState uses its own enum definition of door state/mode which is separated from DoorMode.
  // But their definitions are equal so we can skip conversion.
  return { value: liftState.door_state };
}

const getLiftModeVariant = (
  currentLevel: string,
  liftStateMode?: number,
  liftStateFloor?: string,
): keyof typeof liftMarkerClasses | undefined => {
  if (!liftStateMode && !liftStateFloor) return 'unknown';
  if (liftStateMode === RmfLiftState.MODE_FIRE) return 'fire';
  if (liftStateMode === RmfLiftState.MODE_EMERGENCY) return 'emergency';
  if (liftStateMode === RmfLiftState.MODE_OFFLINE) return 'offLine';
  if (liftStateFloor === currentLevel) {
    if (liftStateMode === RmfLiftState.MODE_HUMAN) return 'human';
    if (liftStateMode === RmfLiftState.MODE_AGV) return 'onCurrentLevel';
  } else {
    if (liftStateMode === RmfLiftState.MODE_HUMAN) return 'moving';
    if (liftStateMode === RmfLiftState.MODE_AGV) return 'moving';
  }
  if (liftStateMode === RmfLiftState.MODE_UNKNOWN) return 'unknown';

  return 'unknown';
};

interface LiftMarkerProps extends Omit<BaseLiftMarkerProps, 'liftState' | 'variant'> {
  lift: Lift;
  currentLevel: string;
}

const LiftMarker = withLabel(({ lift, currentLevel, ...otherProps }: LiftMarkerProps) => {
  const rmf = React.useContext(RmfAppContext);
  const [liftState, setLiftState] = React.useState<LiftState | undefined>(undefined);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.getLiftStateObs(lift.name).subscribe(setLiftState);
    return () => sub.unsubscribe();
  }, [rmf, lift]);

  return (
    <>
      <BaseLiftMarker
        liftState={liftState}
        variant={getLiftModeVariant(
          currentLevel,
          liftState?.current_mode,
          liftState?.current_floor,
        )}
        {...otherProps}
      />
      {lift.doors.map((door: Door, idx: number) => {
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
            doorMode={liftState && toDoorMode(liftState).value}
          />
        );
      })}
    </>
  );
});

export interface LiftsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  currentLevel: string;
  lifts: Lift[];
  hideLabels?: boolean;
  onLiftClick?: (ev: React.MouseEvent, lift: Lift) => void;
}

export const LiftsOverlay = React.memo(
  ({
    lifts,
    hideLabels = false,
    onLiftClick,
    currentLevel,
    ...otherProps
  }: LiftsOverlayProps): JSX.Element => {
    const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);
    const scale = useAutoScale(40);

    return (
      <SVGOverlay viewBox={viewBox} {...otherProps}>
        {lifts.map((lift) => {
          const pos = fromRmfCoords([lift.ref_x, lift.ref_y]);
          return (
            <g key={lift.name}>
              <LiftMarker
                lift={lift}
                onClick={(ev) => onLiftClick && onLiftClick(ev, lift)}
                cx={pos[0]}
                cy={pos[1]}
                width={lift.width}
                height={lift.depth}
                yaw={radiansToDegrees(fromRmfYaw(lift.ref_yaw))}
                currentLevel={currentLevel}
                style={{ transform: `scale(${scale})`, transformOrigin: `${pos[0]}px ${pos[1]}px` }}
                aria-label={lift.name}
                labelText={lift.name}
                labelSourceX={pos[0]}
                labelSourceY={pos[1]}
                labelSourceRadius={Math.min(lift.width / 2, lift.depth / 2)}
                labelArrowLength={Math.max((lift.width / 3) * scale, (lift.depth / 3) * scale)}
                hideLabel={hideLabels}
              />
            </g>
          );
        })}
      </SVGOverlay>
    );
  },
);
