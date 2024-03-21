import { Button, Divider, Grid, styled, Typography, useTheme } from '@mui/material';
import type { TaskState } from 'api-client';
import React from 'react';
import { CircularProgressBar } from './circular-progress-bar';
import { LinearProgressBar } from './linear-progress-bar';

function getTaskStatusDisplay(assignedTask?: string, taskStatus?: string | null) {
  if (assignedTask && !taskStatus) {
    return 'Unknown';
  }
  if (assignedTask && taskStatus) {
    return taskStatus;
  } else {
    return 'No Task';
  }
}

const classes = {
  button: 'robot-info-button',
};
const StyledDiv = styled('div')(() => ({
  [`& .${classes.button}`]: {
    '&:hover': {
      background: 'none',
      cursor: 'default',
    },
  },
}));

type TaskStatus = Required<TaskState>['status'];

export interface RobotInfoProps {
  robotName: string;
  battery?: number;
  assignedTask?: string;
  taskStatus?: TaskStatus;
  taskProgress?: number;
  estFinishTime?: number;
}

const finishedStatus: TaskStatus[] = ['failed', 'completed', 'skipped', 'killed', 'canceled'];

export function RobotInfo({
  robotName,
  battery,
  assignedTask,
  taskStatus,
  taskProgress,
  estFinishTime,
}: RobotInfoProps): JSX.Element {
  const theme = useTheme();
  const hasConcreteEndTime = taskStatus && taskStatus in finishedStatus;

  return (
    <StyledDiv>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {robotName}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <Grid container>
        <Grid container item xs={12} justifyContent="center">
          <Typography variant="h6" gutterBottom sx={{ textTransform: 'capitalize' }}>
            {`Task Progress - ${getTaskStatusDisplay(assignedTask, taskStatus)}`}
          </Typography>
        </Grid>
        <Grid item xs={12}>
          {taskProgress && <LinearProgressBar value={taskProgress * 100} />}
        </Grid>
        <Grid container item xs={12} justifyContent="center">
          <Typography variant="h6" gutterBottom>
            Assigned Tasks
          </Typography>
        </Grid>
        <Grid container item xs={12} justifyContent="center">
          <Button
            disableElevation
            variant="outlined"
            className={classes.button}
            disableRipple={true}
            component="div"
          >
            {assignedTask || '-'}
          </Button>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            Battery
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            <span>{!hasConcreteEndTime && 'Est. '}Finish Time</span>
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <CircularProgressBar
            progress={battery ? +(battery * 100).toFixed(2) : 0}
            strokeColor="#20a39e"
          >
            <Typography variant="h6">{`${battery ? (battery * 100).toFixed(2) : 0}%`}</Typography>
          </CircularProgressBar>
        </Grid>
        <Grid item xs={6}>
          <Button
            size="small"
            disableElevation
            variant="outlined"
            className={classes.button}
            disableRipple={true}
          >
            {estFinishTime !== undefined ? `${new Date(estFinishTime).toLocaleString()}` : '-'}
          </Button>
        </Grid>
      </Grid>
    </StyledDiv>
  );
}
