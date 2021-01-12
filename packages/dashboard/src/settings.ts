import React from 'react';

export enum TrajectoryAnimation {
  None,
  Follow,
}

export interface Settings {
  trajectoryAnimation: TrajectoryAnimation;
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
  };
}

type SaveSettingsAction = {
  /**
   * Saves the provided settings into local storage and triggers a state update.
   *
   */
  type: 'saveSettings';
  settings: Settings;
};

type LoadSettingsAction = {
  /**
   * Loads the state from local storage and triggers a state update.
   */
  type: 'loadSettings';
};

export type SettingsAction = SaveSettingsAction | LoadSettingsAction;

export function settingsReducer(_state: Settings, action: SettingsAction) {
  switch (action.type) {
    case 'saveSettings':
      saveSettings(action.settings);
      return action.settings;
    case 'loadSettings':
      return loadSettings();
  }
}

/**
 * A value for this context MUST be provided, the default value is not type safe.
 */
export const SettingsContext = React.createContext<[Settings, React.Dispatch<SettingsAction>]>(
  null as any,
);
