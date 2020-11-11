export enum TrajectoryAnimation {
  None,
  Fill,
  Follow,
}

export enum AnimationSpeed {
  Slow,
  Normal,
  Fast,
}

export enum TrajectoryDiameter {
  FixSize,
  RobotSize,
}

export enum TrajectoryColor {
  Theme,
  RobotColor,
  Shades,
}

export interface Settings {
  trajectoryAnimation: TrajectoryAnimation;
  trajectoryAnimationSpeed: AnimationSpeed;
  trajectoryDiameter: TrajectoryDiameter;
  trajectoryColor: TrajectoryColor;
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
    trajectoryDiameter: TrajectoryDiameter.RobotSize,
    trajectoryColor: TrajectoryColor.RobotColor,
  };
}
