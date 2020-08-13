import React from 'react';

export enum TrajectoryAnimation {
  None,
  Fill,
  Follow,
  Outline,
}

export enum AnimationSpeed {
  Slow,
  Normal,
  Fast,
}

export enum TrajectoryDiameter {
  Default,
  Robot,
}

export interface Settings {
  trajectoryAnimation: TrajectoryAnimation;
  trajectoryAnimationSpeed: AnimationSpeed;
  trajectoryDiameter: TrajectoryDiameter;
}

export function saveSettings(settings: Settings): void {
  localStorage.setItem('settings', JSON.stringify(settings));
}

export function loadSettings(): Settings {
  const settingsStr = localStorage.getItem('settings');
  if (!settingsStr) {
    return defaultSettings();
  }
  return JSON.parse(settingsStr);
}

export function defaultSettings(): Settings {
  return {
    trajectoryAnimation: TrajectoryAnimation.Follow,
    trajectoryAnimationSpeed: AnimationSpeed.Normal,
    trajectoryDiameter: TrajectoryDiameter.Default,
  };
}

export const SettingsContext = React.createContext(defaultSettings());
