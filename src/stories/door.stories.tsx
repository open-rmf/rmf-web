import React from 'react';
import { Divider, Typography } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

import DoorComponent from './BaseComponents/door-component';
import DoorButton from './BaseComponents/door-panel';
import {
  door,
  doors,
  doorStates,
  componentDisplayStyle,
  defaultStyles,
  StyleTyping,
} from './BaseComponents/Utils';

export default {
  title: 'Door',
};

const doorState = doorStates.main_door;

const singleSlidingDoor = {
  ...door,
  door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
};

const doubleSldingDoor = {
  ...door,
  door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING,
};

const styles: StyleTyping = {
  ...defaultStyles,
  example: {
    display: 'flex',
    justifyContent: 'space-between',
    margin: '1rem 0',
  },
};

const renderInfoPanel = (doorType: string, doorState: string): JSX.Element => {
  return (
    <div style={componentDisplayStyle.modeInfoPanel}>
      <Typography align="center" variant="h5">
        Configurations
      </Typography>

      <div style={componentDisplayStyle.modeInfoItem}>
        <Typography variant="body1">Door Type:</Typography>
        <Typography variant="body1">{doorType}</Typography>
      </div>

      <Divider />

      <div style={componentDisplayStyle.modeInfoItem}>
        <Typography variant="body1">Door Mode:</Typography>
        <Typography variant="body1">{doorState}</Typography>
      </div>

      <Divider />

      <div style={componentDisplayStyle.modeInfoLink}>
        <Typography variant="body1">
          Click
          <a
            style={componentDisplayStyle.aTag}
            href="https://osrf.github.io/romi-js-core-interfaces/classes/door.html#door_type"
          >
            {' '}
            here{' '}
          </a>
          for more details on Door types and
          <a
            style={componentDisplayStyle.aTag}
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

export const DoorPanel = () => (
  <div style={styles.root}>
    <div style={styles.example}>
      <Typography variant="h6">Door State</Typography>
      <Typography variant="h6">Button color and representation</Typography>
    </div>
    <DoorButton doors={doors} doorStates={doorStates} />
  </div>
);
