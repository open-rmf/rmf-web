import React from 'react';
import { makeStyles, Typography, Grid, Paper } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

export interface SystemSummaryTaskStateProps {
  tasks: RomiCore.TaskSummary[];
  onClick?: () => void;
}

export interface TaskSummaryState {
  active: number;
  finish: number;
  failed: number;
  queued: number;
}

const useStyles = makeStyles((theme) => ({
  gridWithClick: {
    padding: '0.4rem 0',
    '&:hover': {
      backgroundColor: theme.palette.grey[100],
      cursor: 'pointer',
    },
  },
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
  header: {
    marginBottom: '0.5rem',
  },
}));

export const SystemSummaryTaskState = (props: SystemSummaryTaskStateProps): JSX.Element => {
  const classes = useStyles();
  const { tasks, onClick } = props;

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

  const summary = getTaskSummary();

  return (
    <React.Fragment>
      <Typography className={classes.header} variant="body1">
        Plans
      </Typography>
      <div aria-label="panel" aria-disabled={!onClick} onClick={onClick}>
        <Grid
          className={onClick ? classes.gridWithClick : classes.grid}
          spacing={2}
          container
          direction="row"
        >
          <Grid item xs={3}>
            <Paper elevation={3} className={getStatusLabel('active')}>
              <Typography align="center" variant="h6">
                {summary.active}
              </Typography>
              <Typography align="center" variant="body1">
                Active
              </Typography>
            </Paper>
          </Grid>

          <Grid item xs={3}>
            <Paper elevation={3} className={getStatusLabel('finish')}>
              <Typography align="center" variant="h6">
                {summary.finish}
              </Typography>
              <Typography align="center" variant="body1">
                Finish
              </Typography>
            </Paper>
          </Grid>

          <Grid item xs={3}>
            <Paper elevation={3} className={getStatusLabel('queued')}>
              <Typography align="center" variant="h6">
                {summary.queued}
              </Typography>
              <Typography align="center" variant="body1">
                Queued
              </Typography>
            </Paper>
          </Grid>

          <Grid item xs={3}>
            <Paper elevation={3} className={getStatusLabel('failed')}>
              <Typography align="center" variant="h6">
                {summary.failed}
              </Typography>
              <Typography align="center" variant="body1">
                Failed
              </Typography>
            </Paper>
          </Grid>
        </Grid>
      </div>
    </React.Fragment>
  );
};
