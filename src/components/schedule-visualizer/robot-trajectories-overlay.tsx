import * as L from 'leaflet';
import React from 'react';
import { Trajectory } from '../../robot-trajectory-manager';
import ColorManager from './colors';
import RobotTrajectory, { RobotTrajectoryProps } from './robot-trajectory';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

export interface RobotTrajectoriesOverlayProps extends SVGOverlayProps {
  trajs: readonly Trajectory[];
  colorManager: Readonly<ColorManager>;
  TrajectoryComponent?: React.ComponentType<RobotTrajectoryProps>;
}

export default function RobotTrajectoriesOverlay(
  props: RobotTrajectoriesOverlayProps,
): React.ReactElement {
  const { trajs, colorManager, ...otherProps } = props;
  const TrajectoryComponent = props.TrajectoryComponent || RobotTrajectory;

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;

  // React.useEffect(() => {
  //   if (!pendingColors.length) {
  //     return;
  //   }
  //   (async () => {
  //     await Promise.all(
  //       pendingColors.map(async robot => colorManager.robotColor(robot.name, robot.model)),
  //     );
  //     setPendingColors([]);
  //   })();
  // });

  // FIXME: hardcode for now, as the source of the footprint is expected to change.
  const footprint = 0.5;

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {trajs.map(traj => (
          <TrajectoryComponent
            key={traj.id}
            trajectory={traj}
            footprint={footprint}
            color="green"
          />
        ))}
      </svg>
    </SVGOverlay>
  );
}
