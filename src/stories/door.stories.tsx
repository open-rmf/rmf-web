import React from 'react';
import { Divider, Typography } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

import DoorComponent from './BaseComponents/door-component';
import { StyleTyping } from './BaseComponents/Utils';

export default {
  title: 'Door',
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

const door = {
  door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
  motion_direction: 1,
  motion_range: -1.571,
  name: 'main_door',
  v1_x: 10.8,
  v1_y: -2.3,
  v2_x: 7.7,
  v2_y: -5.5,
};

const doorState = {
  current_mode: {
    value: RomiCore.DoorMode.MODE_CLOSED,
  },
  door_name: 'main_door',
  door_time: { sec: 0, nanosec: 0 },
};

const singleSlidingDoor = {
  ...door,
  door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
};

const doubleSldingDoor = {
  ...door,
  door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING,
};

const renderInfoPanel = (doorType: string, doorState: string): JSX.Element => {
  return (
    <div style={styles.modeInfoPanel}>
      <Typography align="center" variant="h5">
        Configurations
      </Typography>

      <div style={styles.modeInfoItem}>
        <Typography variant="body1">Door Type:</Typography>
        <Typography variant="body1">{doorType}</Typography>
      </div>

      <Divider />

      <div style={styles.modeInfoItem}>
        <Typography variant="body1">Door Mode:</Typography>
        <Typography variant="body1">{doorState}</Typography>
      </div>

      <Divider />

      <div style={styles.modeInfoLink}>
        <Typography variant="body1">
          Click
          <a
            style={styles.aTag}
            href="https://osrf.github.io/romi-js-core-interfaces/classes/door.html#door_type"
          >
            {' '}
            here{' '}
          </a>
          for more details on Door types and
          <a
            style={styles.aTag}
            href="https://osrf.github.io/romi-js-core-interfaces/classes/doormode.html"
          >
            {' '}
            here{' '}
          </a>
          for more details on Door modes.
        </Typography>
      </div>
    </div>
  );
};

export const SinglePanelDoors = () => (
  <DoorComponent
    door={singleSlidingDoor}
    doorState={doorState}
    currentMode={RomiCore.DoorMode.MODE_CLOSED}
    renderInfoPanel={() => renderInfoPanel('Single Sliding Door', 'Close')}
  />
);

export const DoublePanelDoors = () => (
  <DoorComponent
    door={doubleSldingDoor}
    doorState={doorState}
    currentMode={RomiCore.DoorMode.MODE_CLOSED}
    renderInfoPanel={() => renderInfoPanel('Double Sliding Door', 'Close')}
  />
);

export const MovingDoor = () => (
  <DoorComponent
    door={singleSlidingDoor}
    doorState={doorState}
    currentMode={RomiCore.DoorMode.MODE_MOVING}
    renderInfoPanel={() => renderInfoPanel('Single Sliding Door', 'Moving')}
  />
);

export const ClosingDoor = () => (
  <DoorComponent
    door={singleSlidingDoor}
    doorState={doorState}
    currentMode={RomiCore.DoorMode.MODE_OPEN}
    renderInfoPanel={() => renderInfoPanel('Single Sliding Door', 'Open')}
  />
);
