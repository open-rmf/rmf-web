import { Button, Divider, Grid, makeStyles, Typography, useTheme } from '@material-ui/core';
import { Task } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { taskStateToStr, taskTypeToStr } from '../tasks/utils';
import { rosTimeToJs } from '../utils';
import { CircularProgressBar } from './circular-progress-bar';
import { LinearProgressBar } from './linear-progress-bar';
import { VerboseRobot } from './utils';

const useStyles = makeStyles(() => ({
  button: {
    '&:hover': {
      background: 'none',
      cursor: 'default',
    },
  },
}));

export interface RobotInfoProps {
  robot: VerboseRobot;
}

export function RobotInfo({ robot }: RobotInfoProps): JSX.Element {
  const theme = useTheme();
  const [currentTask, setCurrentTask] = React.useState<Task | undefined>();
  const [hasConcreteEndTime, setHasConcreteEndTime] = React.useState<boolean>(false);
  const classes = useStyles();

  function returnTaskLocations(task: RmfModels.TaskSummary): string {
    switch (taskTypeToStr(task.task_profile.description.task_type.type)) {
      case 'Loop':
        return task.task_profile.description.loop.start_name;
      case 'Delivery':
        return task.task_profile.description.delivery.pickup_place_name;
      default:
        return '-';
    }
  }

  function returnTaskDestinations(task: RmfModels.TaskSummary): string {
    switch (taskTypeToStr(task.task_profile.description.task_type.type)) {
      case 'Loop':
        return task.task_profile.description.loop.finish_name;
      case 'Delivery':
        return task.task_profile.description.delivery.dropoff_place_name;
      case 'Clean':
        return task.task_profile.description.clean.start_waypoint;
      default:
        return '-';
    }
  }

  function assignedTasksToStr(robot: VerboseRobot): string {
    return robot.tasks
      .map((task, index) => {
        if (index !== robot.tasks.length - 1) {
          return task.summary.task_id.concat(' → ');
        } else {
          return task.summary.task_id;
        }
      })
      .join('');
  }

  React.useEffect(() => {
    const concreteTasks = [
      RmfModels.TaskSummary.STATE_CANCELED,
      RmfModels.TaskSummary.STATE_COMPLETED,
      RmfModels.TaskSummary.STATE_FAILED,
    ];

    if (robot.tasks.length > 0) {
      setCurrentTask(robot.tasks[0]);
      if (currentTask) {
        setHasConcreteEndTime(concreteTasks.includes(currentTask.summary.state));
      }
    } else {
      setCurrentTask(undefined);
      setHasConcreteEndTime(false);
    }
  }, [currentTask, robot, setCurrentTask]);

  const taskDetails = React.useMemo(() => {
    if (currentTask) {
      const location = returnTaskLocations(currentTask.summary);
      const destination = returnTaskDestinations(currentTask.summary);
      const assignedTasks = assignedTasksToStr(robot);
      return { location, destination, assignedTasks };
    }
  }, [currentTask, robot]);

  return (
    <div>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {robot.name}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <Grid container>
        <Grid container item xs={12} justify="center">
          <Typography variant="h6" gutterBottom>
            {`Task Progress - ${
              currentTask ? taskStateToStr(currentTask.summary.state) : 'No Task'
            }`}
          </Typography>
        </Grid>
        <Grid item xs={12}>
          {currentTask && (
            <LinearProgressBar value={parseInt(currentTask.progress.status.slice(0, -1)) || 0} />
          )}
        </Grid>
        <Grid container item xs={12} justify="center">
          <Typography variant="h6" gutterBottom>
            Assigned Tasks
          </Typography>
        </Grid>
        <Grid container item xs={12} justify="center">
          <Button
            disableElevation
            variant="outlined"
            className={classes.button}
            disableRipple={true}
            component="div"
          >
            {taskDetails ? taskDetails.assignedTasks : '-'}
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
            {taskDetails ? taskDetails.location : '-'}
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
            {taskDetails ? taskDetails.destination : '-'}
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
          <CircularProgressBar progress={robot.state.battery_percent} strokeColor="#20a39e">
            <Typography variant="h6">{`${robot.state.battery_percent}%`}</Typography>
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
            {currentTask ? rosTimeToJs(currentTask.summary.end_time).toLocaleTimeString() : '-'}
          </Button>
        </Grid>
      </Grid>
    </div>
  );
}
