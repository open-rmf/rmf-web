import React from 'react';
import DashboardTooltip from '../lib/tooltip';
import { Typography } from '@material-ui/core';
import { render, screen, fireEvent } from '@testing-library/react';

describe('Dashboard Tooltip', () => {
  it('does not render the tooltips when disabled', () => {
    render(
      <DashboardTooltip title="test" id="test-tooltip" enabled={false}>
        <Typography variant="h5">Tooltip is disabled</Typography>
      </DashboardTooltip>,
    );

    fireEvent.mouseEnter(screen.getByRole('heading'));

    expect(screen.queryByTestId('tooltip')).toBeNull();
  });
});
