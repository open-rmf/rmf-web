import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Divider, Typography } from '@material-ui/core';

import RobotComponent from './BaseComponents/robot-component';
import ColorManager from '../components/schedule-visualizer/colors';
import { StyleTyping } from './BaseComponents/Utils';

export default {
  title: 'Robot',
};

const styles: StyleTyping = {
  modeInfoPanel: {
    padding: '2rem',
  },
  modeInfoItem: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: '0.5rem',
  },
  modeInfoLink: {
    marginTop: '0.5rem',
    padding: '0.5rem',
  },
  aTag: {
    textDecoration: 'none',
    color: 'rgb(20, 116, 243)',
  },
};

const robotState = {
  battery_percent: 100,
  location: {
    level_name: 'L1',
    t: { sec: 0, nanosec: 0 },
    x: 8,
    y: -4,
    yaw: 0,
  },
  mode: { mode: RomiCore.RobotMode.MODE_IDLE },
  model: '40_hours',
  name: 'LingLing',
  path: [],
  task_id: 'taskA',
};

const colorManager = new ColorManager();

const renderInfoPanel = (robotMode: string, footprint: number): JSX.Element => {
  return (
    <div style={styles.modeInfoPanel}>
      <Typography align="center" variant="h5">
        Configurations
      </Typography>

      <div style={styles.modeInfoItem}>
        <Typography variant="body1">Robot Mode:</Typography>
        <Typography variant="body1">{robotMode}</Typography>
      </div>

      <Divider />

      <div style={styles.modeInfoItem}>
        <Typography variant="body1">footprint:</Typography>
        <Typography variant="body1">{footprint}</Typography>
      </div>

      <Divider />

      <div style={styles.modeInfoLink}>
        <Typography variant="body1">
          Click
          <a
            style={styles.aTag}
            href="https://osrf.github.io/romi-js-core-interfaces/classes/robotmode.html"
          >
            {' '}
            here{' '}
          </a>
          for more details on Robot Modes and
          <a
            style={styles.aTag}
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
