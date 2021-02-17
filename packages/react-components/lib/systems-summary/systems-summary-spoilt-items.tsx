import React from 'react';
import { makeStyles, Typography, Paper } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

interface SpoiltItem {
  errorMessage?: string;
  // a short message about the item name and state in the following format:
  // name - state
  itemNameAndState: string;
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
  spoiltDoorClick?(door: RomiCore.Door): void;
  spoiltLiftClick?(lift: RomiCore.Lift): void;
  spoiltRobotClick?(fleet: string, robot: RomiCore.RobotState): void;
  spoiltDispenserClick?(event: React.MouseEvent, guid: string): void;
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
    spoiltDoorClick,
    spoiltDispenserClick,
    spoiltLiftClick,
    spoiltRobotClick,
  } = props;

  const spoiltItemDetails = (nameAndState: string, error?: string): JSX.Element => {
    return (
      <React.Fragment>
        <Typography color="error" variant="body1">
          {nameAndState}
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
                onClick={() => spoiltDoorClick && spoiltDoorClick(item.door)}
                className={classes.paper}
                key={item.itemNameAndState}
                elevation={3}
              >
                {spoiltItemDetails(item.itemNameAndState)}
              </Paper>
            );
          })
        : null}
      {lifts.length > 0
        ? lifts.map((item) => {
            return (
              <Paper
                onClick={() => spoiltLiftClick && spoiltLiftClick(item.lift)}
                className={classes.paper}
                key={item.itemNameAndState}
                elevation={3}
              >
                {spoiltItemDetails(item.itemNameAndState)}
              </Paper>
            );
          })
        : null}
      {dispensers.length > 0
        ? dispensers.map((item) => {
            return (
              <Paper
                onClick={(e) => spoiltDispenserClick && spoiltDispenserClick(e, item.dispenser)}
                className={classes.paper}
                key={item.itemNameAndState}
                elevation={3}
              >
                {spoiltItemDetails(item.itemNameAndState)}
              </Paper>
            );
          })
        : null}
      {robots.length > 0
        ? robots.map((item) => {
            return (
              <Paper
                onClick={() => spoiltRobotClick && spoiltRobotClick(item.fleet, item.robot)}
                className={classes.paper}
                key={item.itemNameAndState}
                elevation={3}
              >
                {spoiltItemDetails(item.itemNameAndState)}
              </Paper>
            );
          })
        : null}
    </React.Fragment>
  );
};
