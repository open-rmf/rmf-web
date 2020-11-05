import React from 'react';
import DashboardTooltip from '../tooltip';
import { Typography } from '@material-ui/core';
import { createShallow } from '@material-ui/core/test-utils';
import { render, screen, fireEvent } from '@testing-library/react';

const mount = createShallow();

const buildWrapper = (title: string, id: string, enabled: boolean) => {
  const root = mount(
    <DashboardTooltip title={title} id={id} enabled={enabled}>
      <Typography variant="h5">Tooltip is enabled</Typography>
    </DashboardTooltip>,
  );
  return root;
};

describe('Dashboard Tooltip', () => {
  it('renders without crashing', () => {
    const root = buildWrapper('test', 'test-tooltip', true);
    expect(root).toMatchSnapshot();
    root.unmount();
  });

  it('renders the tooltips when enabled', () => {
    render(
      <DashboardTooltip title="test" id="test-tooltip" enabled={true}>
        <Typography variant="h5">Tooltip is enabled</Typography>
      </DashboardTooltip>,
    );

    fireEvent.mouseEnter(screen.getByRole('heading'));

    expect(screen.getByTestId('tooltip')).toBeDefined();
  });

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
