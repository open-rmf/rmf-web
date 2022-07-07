import { ThemeProvider } from '@mui/material';
import CssBaseline from '@mui/material/CssBaseline';
import defaultTheme from '@mui/material/styles/defaultTheme';
import { DecoratorFn } from '@storybook/react';

export const parameters = {
  actions: { argTypesRegex: '^on[A-Z].*' },
  controls: {
    matchers: {
      color: /(background|color)$/i,
      date: /Date$/,
    },
  },
};

const withThemeProvider: DecoratorFn = (Story, context) => {
  return (
    <ThemeProvider theme={defaultTheme}>
      <ThemeProvider theme={defaultTheme}>
        <CssBaseline />
        <Story {...context} />
      </ThemeProvider>
    </ThemeProvider>
  );
};
export const decorators = [withThemeProvider];
