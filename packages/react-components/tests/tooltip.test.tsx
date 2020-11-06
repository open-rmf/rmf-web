import React from 'react';
import DashboardTooltip from '../lib/tooltip';
import { Typography } from '@material-ui/core';
import { render, screen, fireEvent } from '@testing-library/react';

describe('Dashboard Tooltip', () => {
  it('renders the tooltip', () => {
    render(
      <DashboardTooltip title="test" id="test-tooltip">
        <Typography variant="h5">Tooltip is disabled</Typography>
      </DashboardTooltip>,
    );

    fireEvent.mouseEnter(screen.getByRole('heading'));

    expect(screen.getByTestId('tooltip')).toBeTruthy();
  });
});
