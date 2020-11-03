import React from 'react';
import ReactDOM from 'react-dom';
import DashboardTooltip from '../tooltip';
import { IconButton } from '@material-ui/core';
import { Settings as SettingsIcon } from '@material-ui/icons';

describe('Dashboard Tooltip', () => {
  it('renders without crashing', () => {
    const div = document.createElement('div');
    ReactDOM.render(
      <DashboardTooltip title="test" id="test-tooltip" enabled={true}>
        <IconButton>
          <SettingsIcon />
        </IconButton>
      </DashboardTooltip>,
      div,
    );
    expect(div).toMatchSnapshot();
    ReactDOM.unmountComponentAtNode(div);
  });

  it('does not render tooltips when disabled', () => {
    const div = document.createElement('div');
    ReactDOM.render(
      <DashboardTooltip title="test" id="test-tooltip" enabled={false}>
        <IconButton>
          <SettingsIcon />
        </IconButton>
      </DashboardTooltip>,
      div,
    );
    expect(div).toMatchSnapshot();
    ReactDOM.unmountComponentAtNode(div);
  });
});
