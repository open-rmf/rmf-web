export enum ThemeMode {
  Light,
  Dark,
}

export interface Settings {
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
    themeMode: ThemeMode.Light,
  };
}
