import { CssBaseline } from '@mui/material';
import { Decorator, Preview } from '@storybook/react';
import { TestProviders } from 'rmf-dashboard-framework/utils/test-utils.test';

import { LocalizationProvider } from '../src/components/locale';

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
