import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import * as L from 'leaflet';
import React from 'react';
import { ColorContext, robotHash, TrajectoryMarker } from 'react-components';
import { Conflict, Trajectory } from '../../robot-trajectory-manager';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const debug = Debug('ScheduleVisualizer:RobotTrajectoriesOverlay');

export interface RobotTrajectoriesOverlayProps extends SVGOverlayProps {
  /**
   * Must be a record of hashes with key from `robotHash` function.
   *
   * Remarks: `Trajectory` is missing robot model so we need to join it with the robot data.
   */
  robots: Record<string, RomiCore.RobotState>;
  trajectories: Trajectory[];
  conflicts: Conflict[];
  animationScale?: number;
}

/**
 * Contexts: ColorContext
 */
export const RobotTrajectoriesOverlay = React.memo((props: RobotTrajectoriesOverlayProps) => {
  debug('render');

  const { robots, trajectories, conflicts, animationScale = 60000 / 1800, ...otherProps } = props;
  const colorManager = React.useContext(ColorContext);
  const [colors, setColors] = React.useState<Record<number, string>>({});

  const conflictsFlat = React.useMemo(() => conflicts.flat(), [conflicts]);

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;

  React.useEffect(() => {
    // prevent recursive renders as we set colors in the effect.
    if (Object.keys(colors).length === trajectories.length) {
      return;
    }

    (async () => {
      const newColors: Record<string, string> = {};
      await Promise.all(
        trajectories.map(async (traj) => {
          const key = robotHash(traj.robot_name, traj.fleet_name);
          const robot = robots[key];
          if (!robot) {
            console.error(`trajectory "${traj.id}" belongs to unknown robot`);
            return;
          }
          newColors[traj.id] = await colorManager.robotPrimaryColor(
            traj.fleet_name,
            robot.name,
            robot.model,
          );
        }),
      );
      setColors(newColors);
    })();
  });

  // TODO: move this to SchedulerVisualizer
  // React.useEffect(() => {
  //   function getConflictRobotMessage(): string {
  //     let message = '';
  //     conflictRobotNames.forEach((conflictGroup) => {
  //       message += `[${conflictGroup}] `;
  //     });
  //     return message;
  //   }

  //   if (conflicts.length !== 0) {
  //     notificationDispatch &&
  //       notificationDispatch({
  //         message: `Trajectory conflicts between: ${getConflictRobotMessage()}`,
  //         type: 'error',
  //       });
  //   }
  // }, [conflicts, notificationDispatch, conflictRobotNames]);

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {trajectories.map(
          (traj) =>
            colors[traj.id] && (
              <TrajectoryMarker
                key={traj.id}
                trajectory={traj}
                color={colors[traj.id]}
                conflict={conflictsFlat.includes(traj.id)}
                animationScale={animationScale}
              />
            ),
        )}
      </svg>
    </SVGOverlay>
  );
});

export default RobotTrajectoriesOverlay;
