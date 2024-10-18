import { CssBaseline } from '@mui/material';
import { Decorator, Preview } from '@storybook/react';

import { LocalizationProvider } from '../src/components/locale';
import { TestProviders } from '../src/utils/test-utils.test';

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

const withBaseProviders: Decorator = (Story, context) => {
  return (
    <LocalizationProvider>
      <CssBaseline />
      <TestProviders>
        <Story {...context} />
      </TestProviders>
    </LocalizationProvider>
  );
};

export const decorators = [withBaseProviders];
