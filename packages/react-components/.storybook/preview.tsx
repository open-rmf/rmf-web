import { ThemeProvider } from '@material-ui/core';
import { Theme, StyledEngineProvider } from '@material-ui/core/styles';
import defaultTheme from '@material-ui/core/styles/defaultTheme';
import { DecoratorFn } from '@storybook/react';
import { ThemeProvider as EmotionThemeProvider } from 'emotion-theming';
import CssBaseline from '@material-ui/core/CssBaseline';

declare module '@material-ui/styles/defaultTheme' {
  // eslint-disable-next-line @typescript-eslint/no-empty-interface (remove this line if you don't have the rule enabled)
  interface DefaultTheme extends Theme {}
}

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
