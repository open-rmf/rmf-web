import React from 'react';
import { makeStyles, Typography, Paper } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

interface SpoiltItem {
  errorMessage?: string;
  name: string;
  state: string;
}

export interface SpoiltDoor extends SpoiltItem {
  door: RomiCore.Door;
}

export interface SpoiltLift extends SpoiltItem {
  lift: RomiCore.Lift;
}

export interface SpoiltDispenser extends SpoiltItem {
  dispenser: string;
}

export interface SpoiltRobot extends SpoiltItem {
  fleet: string;
  robot: RomiCore.RobotState;
}

export interface SystemSummarySpoiltItemsProps {
  doors: SpoiltDoor[];
  lifts: SpoiltLift[];
  dispensers: SpoiltDispenser[];
  robots: SpoiltRobot[];
  onClickSpoiltDoor?(door: RomiCore.Door): void;
  onClickSpoiltLift?(lift: RomiCore.Lift): void;
  onClickSpoiltRobot?(fleet: string, robot: RomiCore.RobotState): void;
  onClickSpoiltDispenser?(event: React.MouseEvent, guid: string): void;
}

const useStyles = makeStyles((theme) => ({
  paper: {
    padding: theme.spacing(1),
    margin: '1rem 0',
    '&:hover': {
      cursor: 'pointer',
      backgroundColor: theme.palette.grey[100],
    },
  },
}));

export const SystemSummarySpoiltItems = (props: SystemSummarySpoiltItemsProps): JSX.Element => {
  const classes = useStyles();
  const {
    doors,
    lifts,
    robots,
    dispensers,
    onClickSpoiltDoor,
    onClickSpoiltDispenser,
    onClickSpoiltLift,
    onClickSpoiltRobot,
  } = props;

  const spoiltItemDetails = (name: string, state: string, error?: string): JSX.Element => {
    return (
      <React.Fragment>
        <Typography color="error" variant="body1">
          {`${name} - ${state}`}
        </Typography>
        {error !== undefined ? (
          <Typography color="error" variant="body1">
            Error - {error}
          </Typography>
        ) : null}
      </React.Fragment>
    );
  };

  return (
    <React.Fragment>
      <Typography color="error" variant="h6">
        Equipment Out Of Order
      </Typography>
      {doors.length > 0
        ? doors.map((item) => {
            return (
              <Paper
                onClick={() => onClickSpoiltDoor && onClickSpoiltDoor(item.door)}
                className={classes.paper}
                key={'door-' + item.name + '-' + item.state}
                elevation={3}
              >
                {spoiltItemDetails(item.name, item.state)}
              </Paper>
            );
          })
        : null}
      {lifts.length > 0
        ? lifts.map((item) => {
            return (
              <Paper
                onClick={() => onClickSpoiltLift && onClickSpoiltLift(item.lift)}
                className={classes.paper}
                key={'lift-' + item.name + '-' + item.state}
                elevation={3}
              >
                {spoiltItemDetails(item.name, item.state)}
              </Paper>
            );
          })
        : null}
      {dispensers.length > 0
        ? dispensers.map((item) => {
            return (
              <Paper
                onClick={(e) => onClickSpoiltDispenser && onClickSpoiltDispenser(e, item.dispenser)}
                className={classes.paper}
                key={'dispenser-' + item.name + '-' + item.state}
                elevation={3}
              >
                {spoiltItemDetails(item.name, item.state)}
              </Paper>
            );
          })
        : null}
      {robots.length > 0
        ? robots.map((item) => {
            return (
              <Paper
                onClick={() => onClickSpoiltRobot && onClickSpoiltRobot(item.fleet, item.robot)}
                className={classes.paper}
                key={'robot-' + item.name + '-' + item.state}
                elevation={3}
              >
                {spoiltItemDetails(item.name, item.state)}
              </Paper>
            );
          })
        : null}
    </React.Fragment>
  );
};
