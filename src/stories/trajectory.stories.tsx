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

const followAnim = createTrajectories(false, 20);
const partialConflictfollowAnim = {
  ...followAnim,
  conflicts: [[1, 2, 3, 4, 5, 6, 7, 8, 9, 19]],
};
const conflictFollowAnim = createTrajectories(true, 20);

const colorManager = new ColorManager();

const descriptions = {
  themeColor: 'This is a non-conflicting trajectory using material-ui theme color, success.main.',
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
    description={descriptions.themeColor}
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
    description={descriptions.themeColor}
    currSettings={defaultSettings()}
  />
);

export const followAnimation = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={followAnim.conflicts}
    colorManager={colorManager}
    conflictRobotNames={followAnim.conflictingRobotName}
    trajs={followAnim.trajectories}
    description={descriptions.themeColor}
    currSettings={defaultSettings()}
  />
);

export const followAnimationMix = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={partialConflictfollowAnim.conflicts}
    colorManager={colorManager}
    conflictRobotNames={partialConflictfollowAnim.conflictingRobotName}
    trajs={partialConflictfollowAnim.trajectories}
    description={descriptions.themeColor}
    currSettings={defaultSettings()}
  />
);

export const followAnimationConflict = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={conflictFollowAnim.conflicts}
    colorManager={colorManager}
    conflictRobotNames={conflictFollowAnim.conflictingRobotName}
    trajs={conflictFollowAnim.trajectories}
    description={descriptions.themeColor}
    currSettings={defaultSettings()}
  />
);
