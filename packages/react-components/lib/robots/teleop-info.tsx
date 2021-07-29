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
import { LinearProgressBar } from './linear-progress-bar';
import { CircularProgressBar } from './circular-progress-bar';
import * as RmfModels from 'rmf-models';
import { taskTypeToStr, taskStateToStr } from '../tasks/utils';
import { VerboseRobot } from './utils';
import { rosTimeToJs } from '../utils';
import { TaskProgress } from 'api-client';

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

export interface TeleopInfoProps {
  robot: VerboseRobot;
}

export function TeleopInfo({ robot }: TeleopInfoProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();
  const [currentTask, setCurrentTask] = React.useState<TaskProgress | undefined>();
  const [hasConcreteEndTime, setHasConcreteEndTime] = React.useState<boolean>(false);

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
          return task.task_summary.task_id.concat(' → ');
        } else {
          return task.task_summary.task_id;
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
        setHasConcreteEndTime(concreteTasks.includes(currentTask.task_summary.state));
      }
    } else {
      setCurrentTask(undefined);
      setHasConcreteEndTime(false);
    }
  }, [currentTask, robot, setCurrentTask]);

  const taskDetails = React.useMemo(() => {
    if (currentTask) {
      const location = returnTaskLocations(currentTask.task_summary);
      const destination = returnTaskDestinations(currentTask.task_summary);
      const assignedTasks = assignedTasksToStr(robot);
      return { location, destination, assignedTasks };
    }
  }, [currentTask, robot]);

  //if (robot.name === 'teleopRobot') {
  if (true) {
    return (
      <div style={{ height: '100%' }}>
        <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
          Sphero
        </Typography>
        <Divider />
        <iframe width="100%" height="100%" src="https://tiles.demo.open-rmf.org/sphero"></iframe>
      </div>
    );
  } else {
    return (
      <div style={{ height: '100%' }}>
        <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
          {robot.name}
        </Typography>
        <Divider />
        <iframe
          width="560"
          height="315"
          src="https://www.youtube.com/embed/dQw4w9WgXcQ"
          title="YouTube video player"
          frameBorder="0"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
          allowFullScreen
        ></iframe>
      </div>
    );
  }
}
