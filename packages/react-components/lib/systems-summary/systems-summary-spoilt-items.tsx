import React from 'react';
import { makeStyles, Typography, Paper } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

export interface SpoiltItem {
  type: string;
  name: string;
  itemNameAndState: string;
  errorMessage?: string;
  fleet?: string;
}

export interface SystemSummarySpoiltItemsProps {
  spoiltItems: SpoiltItem[];
  doors: RomiCore.Door[];
  lifts: RomiCore.Lift[];
  dispensers: string[];
  robots: Record<string, RomiCore.FleetState>;
  spoiltDoorClick?(door: RomiCore.Door): void;
  spoiltLiftClick?(lift: RomiCore.Lift): void;
  spoiltRobotClick?(fleet: string, robot: RomiCore.RobotState): void;
  spoiltDispenserClick?(event: React.MouseEvent, guid: string): void;
}

const useStyles = makeStyles((theme) => ({
  paper: {
    padding: theme.spacing(1),
    margin: '1rem 0',
  },
}));

export const SystemSummarySpoiltItems = (props: SystemSummarySpoiltItemsProps): JSX.Element => {
  const classes = useStyles();
  const {
    spoiltItems,
    doors,
    lifts,
    robots,
    dispensers,
    spoiltDoorClick,
    spoiltDispenserClick,
    spoiltLiftClick,
    spoiltRobotClick,
  } = props;

  const handlesSpoiltItemClick = (i: number, e?: React.MouseEvent) => {
    const spoiltItem = spoiltItems[i];

    // handle lift
    if (spoiltItem.type === 'lift') {
      lifts.forEach((lift) => {
        if (lift.name === spoiltItem.name && spoiltLiftClick) {
          spoiltLiftClick(lift);
        }
      });
    }

    // handle robot
    if (spoiltItem.type === 'robot') {
      Object.keys(robots).forEach((fleet) => {
        robots[fleet].robots.forEach((robot) => {
          if (spoiltItem.fleet === fleet && spoiltItem.name === robot.name && spoiltRobotClick) {
            spoiltRobotClick(fleet, robot);
          }
        });
      });
    }

    // handle door
    if (spoiltItem.type === 'door') {
      doors.forEach((door) => {
        if (door.name === spoiltItem.name && spoiltDoorClick) {
          spoiltDoorClick(door);
        }
      });
    }

    // handle dispenser
    if (spoiltItem.type === 'dispenser') {
      dispensers.forEach((dispenser) => {
        if (spoiltItem.name === dispenser && spoiltDispenserClick && e) {
          spoiltDispenserClick(e, dispenser);
        }
      });
    }
  };

  return (
    <React.Fragment>
      <Typography color="error" variant="h6">
        Equipment Out Of Order
      </Typography>
      {spoiltItems.map((item, i) => {
        return (
          <Paper
            onClick={(e) => handlesSpoiltItemClick(i, e)}
            className={classes.paper}
            key={item.itemNameAndState}
            elevation={3}
          >
            <Typography color="error" variant="body1">
              {item.itemNameAndState}
            </Typography>
            {item.errorMessage !== undefined ? (
              <Typography color="error" variant="body1">
                Error - {item.errorMessage}
              </Typography>
            ) : null}
          </Paper>
        );
      })}
    </React.Fragment>
  );
};
