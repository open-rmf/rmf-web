import { Theme } from '@mui/material/styles';
import defaultTheme from '@mui/material/styles/defaultTheme';
import { DecoratorFn } from '@storybook/react';
import React from 'react';
import { rmfDark, rmfLight } from '../lib';
import { ThemeProvider } from '../lib/themes';
import { StyledEngineProvider } from '@mui/material/styles';
import { ThemeProvider as EmotionThemeProvider } from 'emotion-theming';
import CssBaseline from '@mui/material/CssBaseline';

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
      return { ...rmfLight };
    case 'rmf-dark':
      return rmfDark;
    default:
      return defaultTheme;
  }
};

const withThemeProvider: DecoratorFn = (Story, context) => {
  const theme = getTheme(context.globals.theme);
  return (
    <StyledEngineProvider injectFirst>
      <EmotionThemeProvider theme={theme}>
        <ThemeProvider theme={theme}>
          <CssBaseline />
          <Story {...context} />
        </ThemeProvider>
      </EmotionThemeProvider>
    </StyledEngineProvider>
  );
};
export const decorators = [withThemeProvider];

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
