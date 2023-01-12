import React from 'react';
import {
  Box,
  Button,
  Grid,
  LinearProgress,
  LinearProgressProps,
  Theme,
  Typography,
  Divider,
  TextField,
} from '@mui/material';
import { base } from 'react-components';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';
import { RobotWithTask } from './task-alert-store';
import { RobotState, Status, Status2, TaskState } from 'api-client';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    title: {
      '&.MuiDialogTitle-root': {
        padding: theme.spacing(1),
        textAlign: 'center',
        fontWeight: 'bold',
      },
    },
    subtitle: {
      '&.MuiTypography-root': {
        fontWeight: 'bold',
      },
    },
    textField: {
      background: theme.palette.background.default,
    },
  }),
);

const LinearProgressWithLabel = (props: LinearProgressProps & { value: number }) => {
  return (
    <Box sx={{ display: 'flex', alignItems: 'center' }}>
      <Box sx={{ width: '100%', mr: 1 }}>
        <LinearProgress variant="determinate" {...props} />
      </Box>
      <Box sx={{ minWidth: 35 }}>
        <Typography variant="body2" color="text.secondary">{`${Math.round(
          props.value * 100,
        )}%`}</Typography>
      </Box>
    </Box>
  );
};

interface AlertToDisplay extends RobotWithTask {
  show: boolean;
}

export interface DialogAlarmProps {
  robotAlert: boolean;
  current: AlertToDisplay;
  setValue: (value: React.SetStateAction<AlertToDisplay[]>) => void;
}

const setTaskDialogColor = (taskStatus: Status | undefined) => {
  if (!taskStatus) {
    return;
  }

  switch (taskStatus) {
    case Status.Failed:
      return base.palette.error.dark;

    case Status.Completed:
      return base.palette.success.dark;

    default:
      return;
  }
};

const setRobotDialogColor = (robotStatus: Status2 | undefined) => {
  if (!robotStatus) {
    return;
  }

  switch (robotStatus) {
    case Status2.Error:
      return base.palette.error.main;

    case Status2.Offline:
      return base.palette.warning.main;

    default:
      return;
  }
};

const showTaskMessage = (task: TaskState) => {
  switch (task.status) {
    case Status.Failed:
      return `${task.dispatch?.status} 
                ${task.dispatch?.errors?.map((e) => e.message)}`;

    case Status.Completed:
      return 'Task completed!';

    default:
      return 'No message';
  }
};

const showRobotMessage = (robot: RobotState) => {
  switch (robot.status) {
    case Status2.Error:
      return 'Robot changed its state to Error.';

    case Status2.Offline:
      return 'Robot changed its state to offline';

    default:
      return 'No message';
  }
};

export const AlertDialog = React.memo((props: DialogAlarmProps) => {
  const classes = useStyles();

  const { setValue, current, robotAlert } = props;

  return (
    <Dialog
      key={current.robot.name}
      PaperProps={{
        style: {
          backgroundColor: robotAlert
            ? setRobotDialogColor(current.robot.status)
            : setTaskDialogColor(current.task?.status),
          boxShadow: 'none',
        },
      }}
      open={current.show}
      maxWidth="xs"
    >
      <DialogTitle className={classes.title}>Alert</DialogTitle>
      <Divider />
      <DialogTitle className={classes.title}>
        {robotAlert ? 'Robot State' : 'Task State'}
      </DialogTitle>
      <Box sx={{ width: '100%' }}>
        <LinearProgressWithLabel value={current.robot.battery ? current.robot.battery : -1} />
      </Box>
      <DialogContent>
        <Grid container mb={1} alignItems="center" spacing={1}>
          <Grid item xs={3}>
            <Typography className={classes.subtitle}>
              {robotAlert ? 'Robot Name' : 'Task'}
            </Typography>
          </Grid>
          <Grid item xs={9}>
            <TextField
              size="small"
              value={robotAlert ? current.robot.name : current.task ? current.task.booking.id : ''}
              InputProps={{
                readOnly: true,
                className: classes.textField,
              }}
            />
          </Grid>
        </Grid>
        <Grid container mb={1} alignItems="center" spacing={1}>
          <Grid item xs={3}>
            <Typography className={classes.subtitle}>Location</Typography>
          </Grid>
          <Grid item xs={9}>
            <TextField
              size="small"
              value={current.robot.location?.map}
              InputProps={{
                readOnly: true,
                className: classes.textField,
              }}
            />
          </Grid>
        </Grid>
        <Grid container alignItems="center" spacing={1}>
          <Grid item xs={3}>
            <Typography className={classes.subtitle}>Message</Typography>
          </Grid>
          <Grid item xs={9}>
            <TextField
              size="small"
              multiline
              value={
                robotAlert
                  ? showRobotMessage(current.robot)
                  : current.task
                  ? showTaskMessage(current.task)
                  : ''
              }
              maxRows={4}
              InputProps={{
                readOnly: true,
                className: classes.textField,
              }}
            />
          </Grid>
        </Grid>
      </DialogContent>
      <DialogActions>
        <Button
          onClick={() =>
            setValue((prev) =>
              prev.map((obj) => {
                if (obj.robot.name === current.robot.name) {
                  return { ...obj, show: false };
                }
                return obj;
              }),
            )
          }
          autoFocus
        >
          Close
        </Button>
      </DialogActions>
    </Dialog>
  );
});
