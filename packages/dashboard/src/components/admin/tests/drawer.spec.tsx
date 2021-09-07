import { ThemeProvider } from '@mui/material';
import { render as render_ } from '@testing-library/react';
import React from 'react';
import { MemoryRouter } from 'react-router';
import { theme } from '../../theme';
import { AdminDrawer } from '../drawer';

const render = (component: React.ReactNode) =>
  render_(
    <ThemeProvider theme={theme}>
      <MemoryRouter>{component}</MemoryRouter>
    </ThemeProvider>,
  );

describe('AdminDrawer', () => {
  it('smoke test', () => {
    render(<AdminDrawer />);
  });
});
