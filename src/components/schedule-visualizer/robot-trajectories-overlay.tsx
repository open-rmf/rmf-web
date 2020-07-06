import * as L from 'leaflet';
import React, { useContext, useEffect } from 'react';
import { Conflict, Trajectory, DefaultTrajectoryManager } from '../../robot-trajectory-manager';
import ColorManager from './colors';
import RobotTrajectory, { RobotTrajectoryProps } from './robot-trajectory';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { NotificationBarContext } from '../notification-bar';

export interface RobotTrajectoriesOverlayProps extends SVGOverlayProps {
  trajs: readonly Trajectory[];
  conflicts: Conflict[];
  colorManager: Readonly<ColorManager>;
}

export default function RobotTrajectoriesOverlay(
  props: RobotTrajectoriesOverlayProps,
): React.ReactElement {
  const { trajs, conflicts, ...otherProps } = props;
  const trajectoryContext = useContext(RobotTrajectoryContext);
  const notificationDispatch = useContext(NotificationBarContext);

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;

  // FIXME: hardcode for now, as the source of the footprint is expected to change.
  const footprint = 0.5;

  useEffect(() => {
    function getConflictRobotsName(): string[] {
      let robotNames: string[] = [];
      conflicts.forEach(conflictId => {
        const robotName = DefaultTrajectoryManager.getRobotNameFromPathId(conflictId, trajs);

        robotName && robotNames.push(robotName);
      });
      return robotNames;
    }

    if (conflicts.length !== 0) {
      notificationDispatch &&
        notificationDispatch({
          message: `Trajectory conflict between ${getConflictRobotsName()}`,
          type: 'error',
        });
    }
  }, [conflicts, notificationDispatch, trajs]);

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {trajs.map(traj => (
          <trajectoryContext.Component
            key={traj.id}
            trajectory={traj}
            footprint={footprint}
            conflicts={conflicts}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
}

export interface RobotTrajectoryContext {
  Component: React.ComponentType<RobotTrajectoryProps>;
}

export const RobotTrajectoryContext = React.createContext<RobotTrajectoryContext>({
  Component: RobotTrajectory,
});
