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

  // function returnTaskLocations(task: TaskSummary): string {
  //   switch (taskTypeToStr(task.task_profile.description.task_type.type)) {
  //     case 'Loop':
  //       return task.task_profile.description.loop.start_name;
  //     case 'Delivery':
  //       return task.task_profile.description.delivery.pickup_place_name;
  //     default:
  //       return '-';
  //   }
  // }

  // function returnTaskDestinations(task: TaskSummary): string {
  //   switch (taskTypeToStr(task.task_profile.description.task_type.type)) {
  //     case 'Loop':
  //       return task.task_profile.description.loop.finish_name;
  //     case 'Delivery':
  //       return task.task_profile.description.delivery.dropoff_place_name;
  //     case 'Clean':
  //       return task.task_profile.description.clean.start_waypoint;
  //     default:
  //       return '-';
  //   }
  // }

  // function assignedTasksToStr(robot: RobotState): string {
  //   return robot.tasks
  //     .map((task, index) => {
  //       if (index !== robot.tasks.length - 1) {
  //         return task.booking.id.concat(' → ');
  //       } else {
  //         return task.booking.id;
  //       }
  //     })
  //     .join('');
  // }

  // React.useEffect(() => {
  //   const concreteTasks = [
  //     RmfTaskSummary.STATE_CANCELED,
  //     RmfTaskSummary.STATE_COMPLETED,
  //     RmfTaskSummary.STATE_FAILED,
  //   ];

  //   if (robot.tasks.length > 0) {
  //     setCurrentTask(robot.tasks[0]);
  //     if (currentTask) {
  //       setHasConcreteEndTime(concreteTasks.includes(currentTask.summary.state));
  //     }
  //   } else {
  //     setCurrentTask(undefined);
  //     setHasConcreteEndTime(false);
  //   }
  // }, [currentTask, robot, setCurrentTask]);

  const taskDetails = React.useMemo(() => {
    // if (currentTask) {
    //   const location = returnTaskLocations(currentTask.summary);
    //   const destination = returnTaskDestinations(currentTask.summary);
    //   const assignedTasks = assignedTasksToStr(robot);
    //   return { location, destination, assignedTasks };
    // }
  }, [currentTask, robot]);

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
          11%
          {/**TODO - figure out a way to calculate task progress
           * One idea is to use the length of pending and completed phases
           * drawback is that there may be some task without any phases so a seperate solution needs to handle such a case
           */}
          {/* {currentTask && (
            <LinearProgressBar value={parseInt(currentTask.progress.status.slice(0, -1)) || 0} />
          )} */}
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
            Location
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            Destination
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <Button
            size="small"
            disableElevation
            variant="outlined"
            className={classes.button}
            disableRipple={true}
          >
            location
            {currentTask ? ` - ${parseTaskDetail(currentTask, currentTask?.category).from}` : '-'}
          </Button>
        </Grid>
        <Grid item xs={6}>
          <Button
            size="small"
            disableElevation
            variant="outlined"
            className={classes.button}
            disableRipple={true}
          >
            destination
            {currentTask ? ` - ${parseTaskDetail(currentTask, currentTask?.category).to}` : '-'}
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
                  new Date(currentTask.estimate_millis * 1000 + Date.now()),
                  "hh:mm aaaaa'm'",
                )}`
              : '-'}
          </Button>
        </Grid>
      </Grid>
    </StyledDiv>
  );
}
