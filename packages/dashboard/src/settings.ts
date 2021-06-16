import { withStyles } from '@material-ui/core/styles';

export enum TrajectoryAnimation {
  None,
  Follow,
}

export enum ThemeMode {
  Light,
  Dark,
}

export interface Settings {
  trajectoryAnimation: TrajectoryAnimation;
  themeMode: ThemeMode;
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
    themeMode: ThemeMode.Light,
  };
}

export const GlobalCss = withStyles((theme) => ({
  '@global': {
    '.MuiDivider-root': {
      backgroundColor: theme.palette.success.main,
    },
    '.MuiAccordionSummary-expandIcon': {
      color: theme.fontColors,
    },
  },
}))(() => null);
