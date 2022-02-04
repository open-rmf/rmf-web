import { Button, Divider, Grid, Typography, useTheme, styled } from '@mui/material';
import type { RobotState, TaskState } from 'api-client';
import React from 'react';
import { taskStateToStr, parseTaskDetail } from '../tasks/utils';
import { format } from 'date-fns';
import { CircularProgressBar } from './circular-progress-bar';
import { LinearProgressBar } from './linear-progress-bar';

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

export interface RobotInfoProps {
  robot: RobotState;
  fetchSelectedTask?: (taskId: string) => Promise<TaskState | undefined>;
}

export function RobotInfo({ robot, fetchSelectedTask }: RobotInfoProps): JSX.Element {
  const theme = useTheme();
  const [currentTask, setCurrentTask] = React.useState<TaskState | undefined>();
  const [hasConcreteEndTime, setHasConcreteEndTime] = React.useState<boolean>(false);

  React.useEffect(() => {
    (async () => {
      if (robot.task_id) {
        fetchSelectedTask && setCurrentTask(await fetchSelectedTask(robot.task_id));
      }
    })();
  });

  return (
    <StyledDiv>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {robot.name}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <Grid container>
        <Grid container item xs={12} justifyContent="center">
          <Typography variant="h6" gutterBottom>
            {`Task Progress - ${currentTask ? taskStateToStr(currentTask) : 'No Task'}`}
          </Typography>
        </Grid>
        <Grid item xs={12}>
          {currentTask &&
          currentTask.unix_millis_start_time &&
          currentTask.unix_millis_finish_time ? (
            <LinearProgressBar
              value={
                (100 * (Date.now() - currentTask.unix_millis_start_time)) /
                (currentTask.unix_millis_finish_time - currentTask.unix_millis_start_time)
              }
            />
          ) : (
            <LinearProgressBar value={0} />
          )}
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
            assigned task
            {robot ? ` - ${robot.task_id}` : '-'}
          </Button>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            Battery
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            <span>{!hasConcreteEndTime && 'Est. '}End Time</span>
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <CircularProgressBar
            progress={robot.battery ? robot.battery * 100 : 0}
            strokeColor="#20a39e"
          >
            <Typography variant="h6">{`${robot.battery ? robot.battery * 100 : 0}%`}</Typography>
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
            time
            {currentTask?.estimate_millis
              ? ` - ${format(
                  new Date(currentTask.estimate_millis * 1 + Date.now()),
                  "hh:mm aaaaa'm'",
                )}`
              : '-'}
          </Button>
        </Grid>
      </Grid>
    </StyledDiv>
  );
}
