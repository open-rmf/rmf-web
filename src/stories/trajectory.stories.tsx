import React from 'react';

import { createTrajectories } from './baseComponents/utils-trajectory';
import {
  defaultTraj,
  conflictingTraj,
  multipleTrajs,
  multipleConflictingTraj,
} from './baseComponents/utils-default-traj';
import Trajectory from './baseComponents/trajectory';
import { mapBound } from './baseComponents/utils';
import ColorManager from '../components/schedule-visualizer/colors';
import { defaultSettings, TrajectoryColor, TrajectoryAnimation, Settings } from '../settings';

export default {
  title: 'Trajectory',
};

// assign 10 trajectories to be conflicting
const partialConflictfollowAnim = {
  ...multipleTrajs,
  conflicts: [[1, 2, 3, 4, 5, 6, 7, 8, 9, 19]],
};
// const conflictFollowAnim = createTrajectories(true, 20);

const colorManager = new ColorManager();

const descriptions = {
  themeColor: 'This is a non-conflicting trajectory using material-ui theme color, success.main.',
};

const themeSettings: Settings = {
  ...defaultSettings(),
  trajectoryColor: TrajectoryColor.Theme,
};

const outlineAnimSettings: Settings = {
  ...defaultSettings(),
  trajectoryAnimation: TrajectoryAnimation.Outline,
};

const fillAnimSettings: Settings = {
  ...defaultSettings(),
  trajectoryAnimation: TrajectoryAnimation.Fill,
};

export const themeColorTrajectory = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={defaultTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={defaultTraj.conflictingRobotName}
    trajs={defaultTraj.trajectories}
    description={descriptions.themeColor}
    currSettings={themeSettings}
  />
);

export const conflictTrajectory = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={conflictingTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={conflictingTraj.conflictingRobotName}
    trajs={conflictingTraj.trajectories}
    description={descriptions.themeColor}
    currSettings={defaultSettings()}
  />
);

export const followAnimation = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={multipleTrajs.conflicts}
    colorManager={colorManager}
    conflictRobotNames={multipleTrajs.conflictingRobotName}
    trajs={multipleTrajs.trajectories}
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
    conflicts={multipleConflictingTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={multipleConflictingTraj.conflictingRobotName}
    trajs={multipleConflictingTraj.trajectories}
    description={descriptions.themeColor}
    currSettings={defaultSettings()}
  />
);
