import { Typography } from '@material-ui/core';
import { fireEvent, render, screen } from '@testing-library/react';
import React from 'react';
import DashboardTooltip from './tooltip';

describe('Dashboard Tooltip', () => {
  it('renders the tooltip', () => {
    render(
      <DashboardTooltip title="test" id="test" enabled={true}>
        <Typography variant="h5">Tooltip is enabled</Typography>
      </DashboardTooltip>,
    );

    fireEvent.mouseEnter(screen.getByRole('heading'));

    expect(screen.getByTestId('test-tooltip')).toBeTruthy();
  });

  it('does not render the tooltip when disabled', () => {
    render(
      <DashboardTooltip title="test" id="test" enabled={false}>
        <Typography variant="h5">Tooltip is disabled</Typography>
      </DashboardTooltip>,
    );

    fireEvent.mouseEnter(screen.getByRole('heading'));

    expect(screen.queryByTestId('test-tooltip')).toBeNull();
  });
});
