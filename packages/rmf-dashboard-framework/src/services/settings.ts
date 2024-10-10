export type ThemeMode = 'default' | 'dark';

export interface Settings {
  themeMode: ThemeMode;
  microAppSettings: { [k: string]: unknown };
}

export function saveSettings(settings: Settings): void {
  localStorage.setItem('settings', JSON.stringify(settings));
}

export function loadSettings(): Settings {
  const settingsStr = localStorage.getItem('settings');
  if (!settingsStr) {
    return defaultSettings;
  }
  return JSON.parse(settingsStr);
}

const defaultSettings: Settings = {
  themeMode: 'default',
  microAppSettings: {},
};
