import React from 'react';
import { makeStyles, Typography, Grid, Paper, Button } from '@material-ui/core';
import NavigateNextIcon from '@material-ui/icons/NavigateNext';
import { SpoiltRobot } from './index';

export interface RobotSummary {
  operational: number;
  idle: number;
  charging: number;
  spoiltRobots: SpoiltRobot[];
}

export interface RobotSummaryStateProps {
  item: string;
  robotSummary: RobotSummary;
  onClick?: () => void;
}

const useStyles = makeStyles((theme) => ({
  paper: {
    padding: theme.spacing(0.2),
  },
  warning: {
    color: theme.palette.error.main,
    border: `2px solid ${theme.palette.error.main}`,
  },
  operational: {
    color: theme.palette.success.main,
    border: `2px solid ${theme.palette.success.main}`,
  },
  idle: {
    color: theme.palette.warning.main,
    border: `2px solid ${theme.palette.warning.main}`,
  },
  charging: {
    color: theme.palette.info.main,
    border: `2px solid ${theme.palette.info.main}`,
  },
  headerGrid: {
    display: 'flex',
    justifyContent: 'space-between',
  },
  button: {
    padding: 0,
  },
  robotGrid: {
    marginTop: theme.spacing(1),
  },
}));

export const RobotSummaryState = (props: RobotSummaryStateProps): JSX.Element => {
  const classes = useStyles();
  const { item, robotSummary, onClick } = props;

  const totalItem = robotSummary.operational + robotSummary.spoiltRobots.length;
  const operationalItem = robotSummary.operational;

  const getOperationalStatusLabel = (total: number, operational: number): string => {
    if (total === operational) {
      return `${classes.paper} ${classes.operational}`;
    } else {
      return `${classes.paper} ${classes.warning}`;
    }
  };

  const getOtherStatusLabel = (mode: string): string => {
    switch (mode) {
      case 'idle':
        return `${classes.paper} ${classes.idle}`;
      case 'charging':
        return `${classes.paper} ${classes.charging}`;
      default:
        return '';
    }
  };

  return (
    <Grid container spacing={1} direction="column">
      <Grid item className={classes.headerGrid}>
        <Typography variant="h6">{item}</Typography>
        <Button className={classes.button} disabled={!onClick} onClick={onClick}>
          <Typography variant="h6">Details </Typography>
          <NavigateNextIcon />
        </Button>
      </Grid>
      <Grid item>
        <Grid container justify="flex-start" direction="row">
          <Grid item xs={12}>
            <Paper className={getOperationalStatusLabel(totalItem, operationalItem)} elevation={3}>
              <Typography noWrap align="center" variant="h6" aria-label={`${item}-operational`}>
                {`${operationalItem}/${totalItem}`}
              </Typography>
              <Typography align="center" variant="body1">
                Operational
              </Typography>
            </Paper>
          </Grid>
        </Grid>
      </Grid>
      <Grid className={classes.robotGrid} item>
        <Grid container justify="flex-start" direction="row" spacing={2}>
          <Grid item xs={6}>
            <Paper className={getOtherStatusLabel('idle')} elevation={3}>
              <Typography noWrap align="center" variant="h6" aria-label={`${item}-idle`}>
                {robotSummary.idle}
              </Typography>
              <Typography align="center" variant="body1">
                Idle
              </Typography>
            </Paper>
          </Grid>
          <Grid item xs={6}>
            <Paper className={getOtherStatusLabel('charging')} elevation={3}>
              <Typography noWrap align="center" variant="h6" aria-label={`${item}-charging`}>
                {robotSummary.charging}
              </Typography>
              <Typography align="center" variant="body1">
                Charging
              </Typography>
            </Paper>
          </Grid>
        </Grid>
      </Grid>
    </Grid>
  );
};
