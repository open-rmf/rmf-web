import React from 'react';
import { makeStyles, Typography, Grid, Paper } from '@material-ui/core';
import { MainMenuTaskStateProps, TaskSummaryState } from './index';

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

export const MainMenuTaskState = (props: MainMenuTaskStateProps): JSX.Element => {
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
        {Object.keys(getTaskSummary()).map((key) => {
          return (
            <Grid key={key} item xs={3}>
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
