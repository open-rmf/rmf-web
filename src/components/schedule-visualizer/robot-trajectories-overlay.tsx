import * as L from 'leaflet';
import React, { useContext, useEffect } from 'react';
import { Conflict, Trajectory, RawKnot } from '../../robot-trajectory-manager';
import ColorManager from './colors';
import RobotTrajectory, { RobotTrajectoryProps } from './robot-trajectory';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { NotificationBarContext } from '../notification-bar';
import RobotConflictTrajectory from './robot-conflict-trajectory';

export interface RobotTrajectoriesOverlayProps extends SVGOverlayProps {
  trajs: readonly Trajectory[];
  conflicts: Conflict[];
  colorManager: Readonly<ColorManager>;
  conflictsSegments?: RawKnot[][];
}

export default function RobotTrajectoriesOverlay(
  props: RobotTrajectoriesOverlayProps,
): React.ReactElement {
  const { trajs, conflicts, conflictsSegments, ...otherProps } = props;
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
    if (conflicts.length !== 0) {
      notificationDispatch &&
        notificationDispatch({
          message: `Conflict between paths ${conflicts}`,
          type: 'error',
        });
    }
  }, [conflicts, notificationDispatch]);
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
        {conflictsSegments?.length !== 0 &&
          conflictsSegments?.map(segments => (
            <RobotConflictTrajectory footprint={footprint} conflictsSegments={segments} />
          ))}
      </svg>
    </SVGOverlay>
  );
}

export interface RobotTrajectoryContext {
  Component: React.ComponentType<RobotTrajectoryProps>;
}

export interface TrajectoryCoords {
  x: number;
  y: number;
}

export const RobotTrajectoryContext = React.createContext<RobotTrajectoryContext>({
  Component: RobotTrajectory,
});
