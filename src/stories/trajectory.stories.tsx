import React from 'react';

import { createTrajectories } from './baseComponents/utils-trajectory';
import Trajectory from './baseComponents/trajectory';
import { mapBound } from './baseComponents/utils';
import ColorManager from '../components/schedule-visualizer/colors';
import { defaultSettings, TrajectoryColor, Settings } from '../settings';

export default {
  title: 'Trajectory',
};

const themeTrajectoryObject = createTrajectories(false, 1);
const conflictTrajectoryObject = createTrajectories(true, 2);
const colorManager = new ColorManager();

const descriptions = {
  themColor: 'This is a non-conflicting trajectory using material-ui theme color, success.main.',
};

const themeSettings: Settings = {
  ...defaultSettings(),
  trajectoryColor: TrajectoryColor.Theme,
};

export const themeColorTrajectory = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={themeTrajectoryObject.conflicts}
    colorManager={colorManager}
    conflictRobotNames={themeTrajectoryObject.conflictingRobotName}
    trajs={themeTrajectoryObject.trajectories}
    description={descriptions.themColor}
    currSettings={themeSettings}
  />
);

export const conflictTrajectory = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={conflictTrajectoryObject.conflicts}
    colorManager={colorManager}
    conflictRobotNames={conflictTrajectoryObject.conflictingRobotName}
    trajs={conflictTrajectoryObject.trajectories}
    description={descriptions.themColor}
    currSettings={defaultSettings()}
  />
);
