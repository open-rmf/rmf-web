import { CssBaseline } from '@mui/material';
import { Theme, ThemeProvider } from '@mui/material/styles';
import defaultTheme from '@mui/material/styles/defaultTheme';
import { DecoratorFn } from '@storybook/react';
import { LocalizationProvider, rmfDark, rmfLight } from '../lib';

export const parameters = {
  actions: { argTypesRegex: '^on[A-Z].*' },
  controls: {
    matchers: {
      color: /(background|color)$/i,
      date: /Date$/,
    },
  },
  s: { argTypesRegex: '^on[A-Z].*' },
};

const getTheme = (themeName: string): Theme => {
  switch (themeName) {
    case 'rmf-light':
      return rmfLight;
    case 'rmf-dark':
      return rmfDark;
    default:
      return defaultTheme;
  }
};

const withThemeProvider: DecoratorFn = (Story, context) => {
  const theme = getTheme(context.globals.theme);
  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <Story {...context} />
    </ThemeProvider>
  );
};

const withLocalization: DecoratorFn = (Story, context) => {
  return (
    <LocalizationProvider>
      <Story {...context} />
    </LocalizationProvider>
  );
};

export const decorators = [withThemeProvider, withLocalization];

export const globalTypes = {
  theme: {
    name: 'Theme',
    description: 'Global theme for components',
    defaultValue: 'rmf-light',
    toolbar: {
      icon: 'circlehollow',
      // Array of plain string values or MenuItem shape (see below)
      items: ['material-default', 'rmf-light', 'rmf-dark'],
    },
  },
};
