import React from 'react';
import { Divider, Typography } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

import LiftComponent from './BaseComponents/lift-component';
import { StyleTyping, lift, liftStates } from './BaseComponents/Utils';

export default {
  title: 'Lift',
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

const defaultLiftStates = liftStates.main_lift;

const AGVState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_AGV,
  door_state: RomiCore.LiftState.DOOR_OPEN,
};

const FIREState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_FIRE,
  current_floor: 'L2',
  destination_floor: 'L4',
  motion_state: RomiCore.LiftState.MOTION_UP,
};

const UNKOWNState = {
  ...defaultLiftStates,
  door_state: RomiCore.LiftState.DOOR_MOVING,
  motion_state: RomiCore.LiftState.MOTION_UNKNOWN,
};

const HUMANState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_HUMAN,
  current_floor: 'L3',
  destination_floor: 'L1',
  motion_state: RomiCore.LiftState.MOTION_DOWN,
};

const OFFLINEState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_OFFLINE,
};

const EMERGENCYState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_EMERGENCY,
};

const renderInfoPanel = (mode: string, doorState: string, motionState: string): JSX.Element => {
  return (
    <div style={styles.modeInfoPanel}>
      <Typography align="center" variant="h5">
        Configurations
      </Typography>

      <div style={styles.modeInfoItem}>
        <Typography variant="body1">Mode:</Typography>
        <Typography variant="body1">{mode}</Typography>
      </div>

      <Divider />

      <div style={styles.modeInfoItem}>
        <Typography variant="body1">Door State:</Typography>
        <Typography variant="body1">{doorState}</Typography>
      </div>

      <Divider />

      <div style={styles.modeInfoItem}>
        <Typography variant="body1">Motion State:</Typography>
        <Typography variant="body1">{motionState}</Typography>
      </div>

      <Divider />

      <div style={styles.modeInfoLink}>
        <Typography variant="body1">
          Click
          <a
            style={styles.aTag}
            href="https://osrf.github.io/romi-js-core-interfaces/classes/liftstate.html"
          >
            {' '}
            here{' '}
          </a>
          for more details of Lift states.
        </Typography>
      </div>
    </div>
  );
};

export const StateAGV = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('AGV', 'Open', 'Stopped')}
    lift={lift}
    currentFloor={AGVState.current_floor}
    liftState={AGVState}
  />
);

export const StateFIRE = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('FIRE', 'Closed', 'Up')}
    lift={lift}
    currentFloor={FIREState.current_floor}
    liftState={FIREState}
  />
);

export const StateUNKNOWN = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('UNKNOWN', 'Moving', 'Unknown')}
    lift={lift}
    currentFloor={UNKOWNState.current_floor}
    liftState={UNKOWNState}
  />
);

export const StateHUMAN = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('HUMAN', 'Closed', 'Down')}
    lift={lift}
    currentFloor={HUMANState.current_floor}
    liftState={HUMANState}
  />
);

export const State_OFFLINE = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('OFFLINE', 'Closed', 'Stopped')}
    lift={lift}
    currentFloor={OFFLINEState.current_floor}
    liftState={OFFLINEState}
  />
);

export const State_EMERGENCY = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('EMERGENCY', 'Closed', 'Stopped')}
    lift={lift}
    currentFloor={EMERGENCYState.current_floor}
    liftState={EMERGENCYState}
  />
);
