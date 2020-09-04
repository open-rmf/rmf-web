import Debug from 'debug';
import * as L from 'leaflet';
import React, { useContext, useEffect } from 'react';
import { Conflict, Trajectory } from '../../robot-trajectory-manager';
import { NotificationBarContext } from '../notification-bar';
import ColorManager from './colors';
import RobotTrajectory, { RobotTrajectoryProps } from './robot-trajectory';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const debug = Debug('ScheduleVisualizer:RobotTrajectoriesOverlay');

export interface RobotTrajectoriesOverlayProps extends SVGOverlayProps {
  trajs: readonly Trajectory[];
  conflicts: Conflict[];
  colorManager: Readonly<ColorManager>;
  conflictRobotNames: string[][];
}

export const RobotTrajectoriesOverlay = React.memo((props: RobotTrajectoriesOverlayProps) => {
  debug('render');

  const { trajs, conflicts, colorManager, conflictRobotNames, ...otherProps } = props;
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
    function getConflictRobotMessage(): string {
      let message = '';
      conflictRobotNames.forEach(conflictGroup => {
        message += `[${conflictGroup}] `;
      });
      return message;
    }

    if (conflicts.length !== 0) {
      notificationDispatch &&
        notificationDispatch({
          message: `Trajectory conflicts between: ${getConflictRobotMessage()}`,
          type: 'error',
        });
    }
  }, [conflicts, notificationDispatch, conflictRobotNames]);

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {trajs.map(traj => (
          <trajectoryContext.Component
            key={traj.id}
            trajectory={traj}
            footprint={footprint}
            conflicts={conflicts}
            colorManager={colorManager}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
});

export default RobotTrajectoriesOverlay;

export interface RobotTrajectoryContext {
  Component: React.ComponentType<RobotTrajectoryProps>;
}

export const RobotTrajectoryContext = React.createContext<RobotTrajectoryContext>({
  Component: RobotTrajectory,
});
