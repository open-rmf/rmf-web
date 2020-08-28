import React from 'react';

import { createSegments, startingTheta } from './baseComponents/utils-trajectory';
import Trajectory from './baseComponents/trajectory';
import { mapBound } from './baseComponents/utils';
import ColorManager from '../components/schedule-visualizer/colors';

export default {
  title: 'Trajectory',
};

const traj = createSegments(
  10,
  -7,
  startingTheta.vertical.value,
  startingTheta.vertical.direction.down,
);
const tempTraj = {
  dimensions: 0.3,
  fleet_name: 'tinyRobot',
  id: 1,
  robot_name: 'tinyRobot1',
  segments: traj,
  shape: 'circle',
};
const colorManager = new ColorManager();
const conflictingRobotNames = [[]];

const descriptions = {
  themColor: 'This is a non-conflicting trajectory using material-ui theme color, success.main.',
};

export const themeColorTrajectory = () => (
  <div>
    <Trajectory
      bounds={mapBound}
      conflicts={[]}
      colorManager={colorManager}
      conflictRobotNames={conflictingRobotNames}
      trajs={[tempTraj]}
      description={descriptions.themColor}
    />
  </div>
);
