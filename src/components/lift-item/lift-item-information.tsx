import { Divider, Typography, makeStyles, ListItem, List } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import DisableableTypography from '../disableable-typography';
import LiftStateManager from '../../lift-state-manager';

interface liftInformationProps {
  lift: RomiCore.Lift;
  liftState?: RomiCore.LiftState;
}

export const LiftInformation = (props: liftInformationProps) => {
  const { lift, liftState } = props;
  const classes = useStyles();

  function renderList(values: string[]): JSX.Element {
    const items = values.map(val => (
      <ListItem key={val} dense className={classes.noPadding}>
        <Typography variant="body1">{val}</Typography>
      </ListItem>
    ));
    return <List>{items}</List>;
  }

  function renderAvailableFloors(liftState?: RomiCore.LiftState): JSX.Element {
    if (!liftState) {
      return (
        <DisableableTypography disabled={!liftState} variant="body1">
          Unknown
        </DisableableTypography>
      );
    }
    return renderList(liftState.available_floors);
  }

  function renderAvailableModes(liftState?: RomiCore.LiftState): JSX.Element {
    if (!liftState) {
      return (
        <DisableableTypography disabled={!liftState} variant="body1">
          Unknown
        </DisableableTypography>
      );
    }
    const modes = Array.from(liftState.available_modes.values());
    return renderList(modes.map(LiftStateManager.liftModeToString));
  }

  return (
    <>
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Location:</Typography>
        <Typography variant="body1">
          {`(${lift.ref_x.toFixed(3)}, ${lift.ref_y.toFixed(3)})`}
        </Typography>
      </div>
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Destination Floor:</Typography>
        <DisableableTypography disabled={!liftState} variant="body1">
          {liftState ? liftState.destination_floor : 'Unknown'}
        </DisableableTypography>
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Available Floors:</Typography>
        {renderAvailableFloors(liftState)}
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Current Mode:</Typography>
        <DisableableTypography disabled={!liftState} variant="body1">
          {liftState ? LiftStateManager.liftModeToString(liftState.current_mode) : 'Unknown'}
        </DisableableTypography>
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Available Modes:</Typography>
        {renderAvailableModes(liftState)}
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Door State:</Typography>
        <DisableableTypography disabled={!liftState} variant="body1">
          {liftState ? LiftStateManager.doorStateToString(liftState.door_state) : 'Unknown'}
        </DisableableTypography>
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Motion State:</Typography>
        <DisableableTypography disabled={!liftState} variant="body1">
          {liftState ? LiftStateManager.motionStateToString(liftState.motion_state) : 'Unknown'}
        </DisableableTypography>
      </div>
    </>
  );
};

const useStyles = makeStyles(theme => ({
  expansionDetailLine: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },

  noPadding: {
    padding: 0,
  },
}));
