import React from 'react';
import { HeaderBar } from '../lib/header-bar';
import { render, screen } from '@testing-library/react';
import { Typography } from '@material-ui/core';

describe('Header Bar', () => {
  it('renders correctly', () => {
    const tabNames = ['Robots', 'Tasks'];
    const root = render(
      <HeaderBar tabNames={tabNames}>
        <Typography variant="caption">Powered by OpenRMF</Typography>
      </HeaderBar>,
    );
    expect(screen.getByText('Robots')).toBeTruthy();
    expect(screen.getByText('Tasks')).toBeTruthy();
    expect(screen.getByText('Powered by OpenRMF')).toBeTruthy();
    root.unmount();
  });
});
