import { ThemeProvider } from '@material-ui/core';
// import { Theme } from '@material-ui/core/styles';
import defaultTheme from '@material-ui/core/styles/defaultTheme';
import { DecoratorFn } from '@storybook/react';

export const parameters = {
  actions: { argTypesRegex: '^on[A-Z].*' },
};

const withThemeProvider: DecoratorFn = (Story, context) => {
  return (
    <ThemeProvider theme={defaultTheme}>
      <Story {...context} />
    </ThemeProvider>
  );
};
export const decorators = [withThemeProvider];
