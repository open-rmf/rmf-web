import React from 'react';
import { makeStyles, Typography, Grid, Paper } from '@material-ui/core';
import { ItemSummaryState } from './main-menu';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

const useStyles = makeStyles((theme) => ({
  grid: {
    padding: '0.4rem 0',
  },
  normal: {
    backgroundColor: theme.palette.success.main,
  },
  failed: {
    backgroundColor: theme.palette.error.main,
  },
  queue: {
    backgroundColor: theme.palette.info.main,
  },
  paperFont: {
    color: 'white',
  },
}));

export interface MainMenuTaskStateProps {
  tasks: RomiCore.TaskSummary[];
}

export const MainMenuTaskState = (props: MainMenuTaskStateProps) => {
  const classes = useStyles();
  const { tasks } = props;

  const getTaskSummary = () => {
    let modeCounter: ItemSummaryState = { active: 0, finish: 0, failed: 0, queued: 0 };
    tasks.forEach((task) => {
      switch (task.state) {
        case 0:
          modeCounter['queued'] += 1;
          break;
        case 1:
          modeCounter['active'] += 1;
          break;
        case 2:
          modeCounter['finish'] += 1;
          break;
        case 3:
          modeCounter['failed'] += 1;
          break;
      }
    });
    return modeCounter;
  };

  const getStatusLabel = (mode: string): string => {
    switch (mode) {
      case 'active':
      case 'finish':
        return `${classes.normal} ${classes.paperFont}`;
      case 'queued':
        return `${classes.queue} ${classes.paperFont}`;
      case 'failed':
        return `${classes.failed} ${classes.paperFont}`;
      default:
        return '';
    }
  };

  return (
    <React.Fragment>
      <Typography variant="body1">Plans</Typography>
      <Grid className={classes.grid} spacing={2} container direction="row">
        {Object.keys(getTaskSummary()).map((key) => {
          return (
            <Grid item xs={3}>
              <Paper elevation={3} className={getStatusLabel(key)}>
                <Typography align="center" variant="h6">
                  {getTaskSummary()[key]}
                </Typography>
                <Typography align="center" variant="body1">
                  {key}
                </Typography>
              </Paper>
            </Grid>
          );
        })}
      </Grid>
    </React.Fragment>
  );
};
