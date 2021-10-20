import { ThemeProvider } from '@mui/material';
import { StyledEngineProvider } from '@mui/material/styles';
import defaultTheme from '@mui/material/styles/defaultTheme';
import { DecoratorFn } from '@storybook/react';
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
};

const withThemeProvider: DecoratorFn = (Story, context) => {
  return (
    <StyledEngineProvider injectFirst>
      <EmotionThemeProvider theme={defaultTheme}>
        <ThemeProvider theme={defaultTheme}>
          <CssBaseline />
          <Story {...context} />
        </ThemeProvider>
      </EmotionThemeProvider>
    </StyledEngineProvider>
  );
};
export const decorators = [withThemeProvider];
