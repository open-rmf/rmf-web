import type { Preview } from '@storybook/react';

const preview: Preview = {
  parameters: {
    controls: {
      matchers: {
        color: /(background|color)$/i,
        date: /Date$/i,
      },
    },
  },
};

export default preview;

// const withThemeProvider: DecoratorFn = (Story, context) => {
//   return (
//     <ThemeProvider theme={defaultTheme}>
//       <ThemeProvider theme={defaultTheme}>
//         <CssBaseline />
//         <Story {...context} />
//       </ThemeProvider>
//     </ThemeProvider>
//   );
// };
// export const decorators = [withThemeProvider];
