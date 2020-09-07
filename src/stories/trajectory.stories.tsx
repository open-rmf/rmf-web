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
import { defaultSettings, TrajectoryAnimation, Settings } from '../settings';

export default {
  title: 'Trajectory',
};

// assign 10 trajectories to be conflicting
const partialConflictTraj = {
  ...multipleTrajs,
  conflicts: [[1, 2, 3, 4, 5, 6, 7, 8, 9, 19]],
};
const spam = createTrajectories(true, 100);

const colorManager = new ColorManager();

const descriptions = {
  normal: 'A normal trajectory',
  conflictingTraj: 'Conflicting trajectory',
  followAnim: 'Trajectory with follow animation',
  followAnimMixConflict: 'Follow animation with some trajectories conflicting',
  followAnimConflict: 'Follow animation with all trajectories conflicting',
  outlineAnim: 'Trajectory with outline animation',
  outlineAnimMixConflict: 'Outline animation with some trajectories conflicting',
  outlineAnimConflict: 'Outline animation with all trajectories conflicting',
  fillAnim: 'Trajectory with fill animation',
  fillAnimMixConflict: 'Fill animation with some trajectories conflicting',
  fillAnimConflict: 'Fill animation with all trajectories conflicting',
  spam: 'Spam trajectories to test performance (Currently 100 conflicting trajectories)',
};

const outlineAnimSettings: Settings = {
  ...defaultSettings(),
  trajectoryAnimation: TrajectoryAnimation.Outline,
};

const fillAnimSettings: Settings = {
  ...defaultSettings(),
  trajectoryAnimation: TrajectoryAnimation.Fill,
};

export const trajectory = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={defaultTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={defaultTraj.conflictingRobotName}
    trajs={defaultTraj.trajectories}
    description={descriptions.normal}
    currSettings={defaultSettings()}
  />
);

export const conflictTrajectory = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={conflictingTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={conflictingTraj.conflictingRobotName}
    trajs={conflictingTraj.trajectories}
    description={descriptions.conflictingTraj}
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
    description={descriptions.followAnim}
    currSettings={defaultSettings()}
  />
);

export const followAnimationMix = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={partialConflictTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={partialConflictTraj.conflictingRobotName}
    trajs={partialConflictTraj.trajectories}
    description={descriptions.followAnimMixConflict}
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
    description={descriptions.followAnimConflict}
    currSettings={defaultSettings()}
  />
);

export const outlineAnimation = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={multipleTrajs.conflicts}
    colorManager={colorManager}
    conflictRobotNames={multipleTrajs.conflictingRobotName}
    trajs={multipleTrajs.trajectories}
    description={descriptions.outlineAnim}
    currSettings={outlineAnimSettings}
  />
);

export const outlineAnimationMix = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={partialConflictTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={partialConflictTraj.conflictingRobotName}
    trajs={partialConflictTraj.trajectories}
    description={descriptions.outlineAnimMixConflict}
    currSettings={outlineAnimSettings}
  />
);

export const outlineAnimationConflict = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={multipleConflictingTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={multipleConflictingTraj.conflictingRobotName}
    trajs={multipleConflictingTraj.trajectories}
    description={descriptions.outlineAnimConflict}
    currSettings={outlineAnimSettings}
  />
);

export const fillAnimation = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={multipleTrajs.conflicts}
    colorManager={colorManager}
    conflictRobotNames={multipleTrajs.conflictingRobotName}
    trajs={multipleTrajs.trajectories}
    description={descriptions.fillAnim}
    currSettings={fillAnimSettings}
  />
);

export const fillAnimationMix = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={partialConflictTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={partialConflictTraj.conflictingRobotName}
    trajs={partialConflictTraj.trajectories}
    description={descriptions.fillAnimMixConflict}
    currSettings={fillAnimSettings}
  />
);

export const fillAnimationConflict = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={multipleConflictingTraj.conflicts}
    colorManager={colorManager}
    conflictRobotNames={multipleConflictingTraj.conflictingRobotName}
    trajs={multipleConflictingTraj.trajectories}
    description={descriptions.fillAnimConflict}
    currSettings={fillAnimSettings}
  />
);

export const spamTrajectories = () => (
  <Trajectory
    bounds={mapBound}
    conflicts={spam.conflicts}
    colorManager={colorManager}
    conflictRobotNames={spam.conflictingRobotName}
    trajs={spam.trajectories}
    description={descriptions.spam}
    currSettings={defaultSettings()}
  />
);
