import React from 'react';
import { Divider, Typography } from '@material-ui/core';

import RobotComponent from './BaseComponents/robot-component';
import ColorManager from '../components/schedule-visualizer/colors';
import { robotState, componentDisplayStyle } from './BaseComponents/Utils';

export default {
  title: 'Robot',
};

const colorManager = new ColorManager();

const renderInfoPanel = (robotMode: string, footprint: number): JSX.Element => {
  return (
    <div style={componentDisplayStyle.modeInfoPanel}>
      <Typography align="center" variant="h5">
        Configurations
      </Typography>

      <div style={componentDisplayStyle.modeInfoItem}>
        <Typography variant="body1">Robot Mode:</Typography>
        <Typography variant="body1">{robotMode}</Typography>
      </div>

      <Divider />

      <div style={componentDisplayStyle.modeInfoItem}>
        <Typography variant="body1">footprint:</Typography>
        <Typography variant="body1">{footprint}</Typography>
      </div>

      <Divider />

      <div style={componentDisplayStyle.modeInfoLink}>
        <Typography variant="body1">
          Click
          <a
            style={componentDisplayStyle.aTag}
            href="https://osrf.github.io/romi-js-core-interfaces/classes/robotmode.html"
          >
            {' '}
            here{' '}
          </a>
          for more details on Robot Modes and
          <a
            style={componentDisplayStyle.aTag}
            href="https://osrf.github.io/romi-js-core-interfaces/classes/robotstate.html"
          >
            {' '}
            here{' '}
          </a>
          for more details on Robot States.
        </Typography>
      </div>
    </div>
  );
};

export const Robot = () => (
  <RobotComponent
    robot={robotState}
    footprint={1}
    colorManager={colorManager}
    renderInfoPanel={() => renderInfoPanel('Idle', 1)}
  />
);
