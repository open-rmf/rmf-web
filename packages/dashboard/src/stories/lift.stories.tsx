import { Divider, Typography } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import LiftsPanel from '../components/lift-item/lifts-panel';
import LiftComponent from './baseComponents/lift-component';
import {
  componentDisplayStyle,
  defaultStyles,
  lift,
  lifts,
  liftStates,
  StyleTyping,
} from './baseComponents/utils';

export default {
  title: 'Lift',
};

const defaultLiftStates = liftStates.main_lift;

const agvState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_AGV,
  door_state: RomiCore.LiftState.DOOR_OPEN,
};

const fireState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_FIRE,
  current_floor: 'L2',
  destination_floor: 'L4',
  motion_state: RomiCore.LiftState.MOTION_UP,
};

const unknownState = {
  ...defaultLiftStates,
  door_state: RomiCore.LiftState.DOOR_MOVING,
  motion_state: RomiCore.LiftState.MOTION_UNKNOWN,
};

const humanState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_HUMAN,
  current_floor: 'L3',
  destination_floor: 'L1',
  motion_state: RomiCore.LiftState.MOTION_DOWN,
};

const offlineState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_OFFLINE,
};

const emergencyState = {
  ...defaultLiftStates,
  current_mode: RomiCore.LiftState.MODE_EMERGENCY,
};

const styles: StyleTyping = {
  ...defaultStyles,
  example: {
    display: 'flex',
    justifyContent: 'space-between',
    margin: '1rem 0',
  },
};

const renderInfoPanel = (mode: string, doorState: string, motionState: string): JSX.Element => {
  return (
    <div style={componentDisplayStyle.modeInfoPanel}>
      <Typography align="center" variant="h5">
        Configurations
      </Typography>

      <div style={componentDisplayStyle.modeInfoItem}>
        <Typography variant="body1">Mode:</Typography>
        <Typography variant="body1">{mode}</Typography>
      </div>

      <Divider />

      <div style={componentDisplayStyle.modeInfoItem}>
        <Typography variant="body1">Door State:</Typography>
        <Typography variant="body1">{doorState}</Typography>
      </div>

      <Divider />

      <div style={componentDisplayStyle.modeInfoItem}>
        <Typography variant="body1">Motion State:</Typography>
        <Typography variant="body1">{motionState}</Typography>
      </div>

      <Divider />

      <div style={componentDisplayStyle.modeInfoLink}>
        <Typography variant="body1">
          Click
          <a
            style={componentDisplayStyle.aTag}
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

export const stateAgv = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('AGV', 'Open', 'Stopped')}
    lift={lift}
    currentFloor={agvState.current_floor}
    liftState={agvState}
  />
);

export const stateFire = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('FIRE', 'Closed', 'Up')}
    lift={lift}
    currentFloor={fireState.current_floor}
    liftState={fireState}
  />
);

export const stateUnknown = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('UNKNOWN', 'Moving', 'Unknown')}
    lift={lift}
    currentFloor={unknownState.current_floor}
    liftState={unknownState}
  />
);

export const stateHuman = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('HUMAN', 'Closed', 'Down')}
    lift={lift}
    currentFloor={humanState.current_floor}
    liftState={humanState}
  />
);

export const stateOffline = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('OFFLINE', 'Closed', 'Stopped')}
    lift={lift}
    currentFloor={offlineState.current_floor}
    liftState={offlineState}
  />
);

export const stateEmergency = () => (
  <LiftComponent
    renderInfoPanel={() => renderInfoPanel('EMERGENCY', 'Closed', 'Stopped')}
    lift={lift}
    currentFloor={emergencyState.current_floor}
    liftState={emergencyState}
  />
);

export const liftPanel = () => (
  <div style={styles.root}>
    <div style={styles.example}>
      <Typography variant="h6">Lift State</Typography>
      <Typography variant="h6">Button color and representation</Typography>
    </div>
    <LiftsPanel lifts={lifts} liftStates={liftStates} />
  </div>
);
