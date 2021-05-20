import React from 'react';
import {
  Button,
  createStyles,
  Divider,
  Grid,
  makeStyles,
  Typography,
  useTheme,
} from '@material-ui/core';
import { ProgressBar } from '../progressbar';
import { CircularProgressBar } from '../circular-progress-bar';
import * as RmfModels from 'rmf-models';
import { taskTypeToStr } from '../tasks/utils';
import { VerboseRobot } from './utils';
import { rosTimeToJs } from '../utils';

const useStyles = makeStyles((theme) =>
  createStyles({
    root: {
      '&$disabled': {
        color: theme.palette.primary.main,
        borderColor: theme.palette.primary.main,
      },
    },
    disabled: {},
    logo: {
      maxWidth: 120,
      opacity: 1,
    },
    infoValue: {
      float: 'right',
      textAlign: 'right',
    },
    indicator: {
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      textAlign: 'center',
      position: 'absolute',
      top: 0,
      width: '100%',
      height: '100%',
      margin: '0 auto',
    },
  }),
);

export interface RobotInfoProps {
  robot: VerboseRobot;
}

export function RobotInfo({ robot }: RobotInfoProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();
  let currentTask: RmfModels.TaskSummary | undefined = undefined;
  let hasConcreteEndTime = false;

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
    return robot.assigned_tasks
      .map((task, index) => {
        if (index != robot.assigned_tasks.length - 1) {
          return task.task_id + ' → ';
        } else {
          return task.task_id;
        }
      })
      .join('');
  }

  if (robot.assigned_tasks.length > 0) {
    currentTask = robot.assigned_tasks[0];
    hasConcreteEndTime = [
      RmfModels.TaskSummary.STATE_CANCELED,
      RmfModels.TaskSummary.STATE_COMPLETED,
      RmfModels.TaskSummary.STATE_FAILED,
    ].includes(currentTask.state);
  } else {
    currentTask = undefined;
    hasConcreteEndTime = false;
  }

  return (
    <div>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {robot.name}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <Grid container>
        <Grid item xs={12}>
          <Typography variant="h6" style={{ textAlign: 'left' }} gutterBottom>
            Battery
          </Typography>
        </Grid>
        <Grid item xs={12}>
          <ProgressBar value={robot.battery_percent} />
        </Grid>
        <Grid item xs={12}>
          <Typography variant="h6" style={{ textAlign: 'left' }} gutterBottom>
            Assigned Tasks
          </Typography>
        </Grid>
        <Grid item xs={12}>
          <Button
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            {currentTask ? assignedTasksToStr(robot) : '-'}
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
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            {currentTask ? returnTaskLocations(currentTask) : '-'}
          </Button>
        </Grid>
        <Grid item xs={6}>
          <Button
            size="small"
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            {currentTask ? returnTaskDestinations(currentTask) : '-'}
          </Button>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            Task Progress
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            <span>{!hasConcreteEndTime && 'Est. '}End Time</span>
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <CircularProgressBar progress={90} strokeColor="#5CCDBA">
            <div className={classes.indicator}>
              <Typography variant="h6">90%</Typography>
            </div>
          </CircularProgressBar>
        </Grid>
        <Grid item xs={6}>
          <Button
            size="small"
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            {currentTask ? rosTimeToJs(currentTask.end_time).toLocaleTimeString() : '-'}
          </Button>
        </Grid>
      </Grid>
    </div>
  );
}
