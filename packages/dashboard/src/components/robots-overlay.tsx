import { RobotState } from 'api-client';
import React from 'react';
import {
  fromRmfCoords,
  fromRmfYaw,
  RobotMarker as BaseRobotMarker,
  RobotMarkerProps as BaseRobotMarkerProps,
  SVGOverlay,
  SVGOverlayProps,
  useAutoScale,
  viewBoxFromLeafletBounds,
  withLabel,
  WithLabelProps,
} from 'react-components';
import { EMPTY, mergeMap, of } from 'rxjs';
import { RmfAppContext } from './rmf-app';

export interface RobotData {
  fleet: string;
  name: string;
  model: string;
  footprint: number;
  color: string;
  inConflict?: boolean;
  iconPath?: string;
}

const MarkerWithLabel = withLabel(BaseRobotMarker);
type MarkerWithLabelProps = WithLabelProps<BaseRobotMarkerProps>;

interface RobotMarkerProps
  extends Omit<MarkerWithLabelProps, 'style' | 'cx' | 'cy' | 'labelSourceX' | 'labelSourceY'> {
  robot: RobotData;
  scale: number;
}

const RobotMarker = ({ robot, scale, ...otherProps }: RobotMarkerProps) => {
  const rmf = React.useContext(RmfAppContext);
  const [robotState, setRobotState] = React.useState<RobotState | undefined>(undefined);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf
      .getFleetStateObs(robot.fleet)
      .pipe(
        mergeMap((state) =>
          state.robots && state.robots[robot.name] ? of(state.robots[robot.name]) : EMPTY,
        ),
      )
      .subscribe(setRobotState);
    return () => sub.unsubscribe();
  }, [rmf, robot]);

  const [x, y] = robotState?.location
    ? fromRmfCoords([robotState.location.x, robotState.location.y])
    : [0, 0];
  const theta = robotState?.location ? fromRmfYaw(robotState.location.yaw) : 0;

  return (
    <MarkerWithLabel
      cx={x}
      cy={y}
      style={{
        transform: `rotate(${theta}rad) scale(${scale})`,
        transformOrigin: `${x}px ${y}px`,
      }}
      labelSourceX={x}
      labelSourceY={y}
      {...otherProps}
    />
  );
};

export interface RobotsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  robots: RobotData[];
  /**
   * The zoom level at which the markers should transition from actual size to fixed size.
   */
  markerActualSizeMinZoom?: number;
  hideLabels?: boolean;
  onRobotClick?: (ev: React.MouseEvent, robot: RobotData) => void;
}

export const RobotsOverlay = React.memo(
  ({
    robots,
    hideLabels = false,
    onRobotClick,
    ...otherProps
  }: RobotsOverlayProps): JSX.Element => {
    const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);
    const scale = useAutoScale(40);
    // TODO: hardcoded because footprint is not available in rmf.
    const footprint = 0.5;

    return (
      <SVGOverlay viewBox={viewBox} {...otherProps}>
        {robots.map((robot) => {
          return (
            <RobotMarker
              key={`${robot.fleet}/${robot.name}`}
              robot={robot}
              scale={scale}
              r={footprint}
              color={robot.color}
              iconPath={robot.iconPath}
              onClick={(ev) => onRobotClick && onRobotClick(ev, robot)}
              aria-label={robot.name}
              labelText={robot.name}
              labelSourceRadius={footprint * scale}
              hideLabel={hideLabels}
            />
          );
        })}
      </SVGOverlay>
    );
  },
);
