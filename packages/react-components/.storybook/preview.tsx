import { makeStyles, ThemeProvider } from '@material-ui/core';
import { Theme } from '@material-ui/core/styles';
import defaultTheme from '@material-ui/core/styles/defaultTheme';
import { DecoratorFn } from '@storybook/react';
import React from 'react';
import { GlobalDarkCss, rmfDark, rmfLight } from '../lib';

export const parameters = {
  actions: { argTypesRegex: '^on[A-Z].*' },
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

const useStyles = makeStyles((theme) => ({
  '@global': {
    body: {
      backgroundColor: theme.palette.background.paper,
    },
  },
}));

function StorybookBodyStyles() {
  useStyles();
  return null;
}

const withThemeProvider: DecoratorFn = (Story, context) => {
  const theme = getTheme(context.globals.theme);
  useStyles();
  return (
    <ThemeProvider theme={theme}>
      {context.globals.theme === 'rmf-dark' ? <GlobalDarkCss /> : null}
      <StorybookBodyStyles />
      <Story {...context} />
    </ThemeProvider>
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
