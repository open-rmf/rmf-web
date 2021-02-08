import React from 'react';
import { makeStyles, Typography, Grid, Paper } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

export interface SystemSummaryTaskStateProps {
  tasks: RomiCore.TaskSummary[];
}

export interface TaskSummaryState {
  active: number;
  finish: number;
  failed: number;
  queued: number;
}

const useStyles = makeStyles((theme) => ({
  grid: {
    padding: '0.4rem 0',
  },
  normal: {
    color: theme.palette.success.main,
    border: `2px solid ${theme.palette.success.main}`,
  },
  failed: {
    color: theme.palette.error.main,
    border: `2px solid ${theme.palette.error.main}`,
  },
  queue: {
    color: theme.palette.info.main,
    border: `2px solid ${theme.palette.info.main}`,
  },
}));

export const SystemSummaryTaskState = (props: SystemSummaryTaskStateProps): JSX.Element => {
  const classes = useStyles();
  const { tasks } = props;

  const getTaskSummary = () => {
    const modeCounter: TaskSummaryState = { active: 0, finish: 0, failed: 0, queued: 0 };
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
        return classes.normal;
      case 'queued':
        return classes.queue;
      case 'failed':
        return classes.failed;
      default:
        return '';
    }
  };

  return (
    <React.Fragment>
      <Typography variant="body1">Plans</Typography>
      <Grid className={classes.grid} spacing={2} container direction="row">
        <Grid key={'active'} item xs={3}>
          <Paper elevation={3} className={getStatusLabel('active')}>
            <Typography align="center" variant="h6">
              {getTaskSummary().active}
            </Typography>
            <Typography align="center" variant="body1">
              Active
            </Typography>
          </Paper>
        </Grid>

        <Grid key={'finish'} item xs={3}>
          <Paper elevation={3} className={getStatusLabel('finish')}>
            <Typography align="center" variant="h6">
              {getTaskSummary().finish}
            </Typography>
            <Typography align="center" variant="body1">
              Finish
            </Typography>
          </Paper>
        </Grid>

        <Grid key={'queued'} item xs={3}>
          <Paper elevation={3} className={getStatusLabel('queued')}>
            <Typography align="center" variant="h6">
              {getTaskSummary().queued}
            </Typography>
            <Typography align="center" variant="body1">
              Queued
            </Typography>
          </Paper>
        </Grid>

        <Grid key={'failed'} item xs={3}>
          <Paper elevation={3} className={getStatusLabel('failed')}>
            <Typography align="center" variant="h6">
              {getTaskSummary().failed}
            </Typography>
            <Typography align="center" variant="body1">
              Failed
            </Typography>
          </Paper>
        </Grid>
      </Grid>
    </React.Fragment>
  );
};
