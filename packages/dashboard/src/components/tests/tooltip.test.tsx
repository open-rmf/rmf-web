import React from 'react';
import ReactDOM from 'react-dom';
import DashboardTooltip from '../tooltip';
import { IconButton } from '@material-ui/core';
import { Settings as SettingsIcon } from '@material-ui/icons';

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
  ReactDOM.unmountComponentAtNode(div);
});

//add snapshot and enabled={false} test
