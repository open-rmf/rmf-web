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
import { taskTypeToStr } from '../tasks/utils';
import { VerboseRobot } from './utils';

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
  }),
);

export interface RobotInfoProps {
  robot: VerboseRobot;
}

export function RobotInfo({ robot }: RobotInfoProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();
  const currentTask = robot.assigned_tasks[0];
  const currentTaskType = currentTask.task_profile.description.task_type.type;

  const assignedTasksToStr = robot.assigned_tasks
    .map((task, index) => {
      if (index != robot.assigned_tasks.length - 1) {
        return task.task_id + ' → ';
      } else {
        return task.task_id;
      }
    })
    .join('');

  function returnTaskLocations() {
    switch (taskTypeToStr(currentTaskType)) {
      case 'Loop':
        return (
          <Button
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            {currentTask.task_profile.description.loop.start_name}
          </Button>
        );
      case 'Delivery':
        return (
          <Button
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            {currentTask.task_profile.description.delivery.pickup_place_name}
          </Button>
        );
      default:
        return (
          <Button
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            -
          </Button>
        );
    }
  }

  function returnTaskDestinations() {
    switch (taskTypeToStr(currentTaskType)) {
      case 'Loop':
        return (
          <Button
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            {currentTask.task_profile.description.loop.finish_name}
          </Button>
        );
      case 'Delivery':
        return (
          <Button
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            {currentTask.task_profile.description.delivery.dropoff_place_name}
          </Button>
        );
      case 'Clean':
        return (
          <Button
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            {currentTask.task_profile.description.clean.start_waypoint}
          </Button>
        );
      default:
        return (
          <Button
            disableElevation
            variant="outlined"
            classes={{ root: classes.root, disabled: classes.disabled }}
            disabled
          >
            -
          </Button>
        );
    }
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
            {assignedTasksToStr}
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
          {returnTaskLocations()}
        </Grid>
        <Grid item xs={6}>
          {returnTaskDestinations()}
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            Task Progress
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            Est. End Time
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6">-</Typography>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6">-</Typography>
        </Grid>
      </Grid>
    </div>
  );
}
