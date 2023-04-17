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
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';
import { Status, TaskState } from 'api-client';
import { base } from 'react-components';
import { TaskInspector } from './task-inspector';

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

const getTaskProgress = (task: TaskState | null): number => {
  if (!task) {
    return 0;
  }
  if (!task.phases || !task.active) {
    return 0;
  }
  const lastPhase = Object.keys(task.phases).pop();

  if (lastPhase) {
    return (task.active / parseInt(lastPhase)) * 100;
  }

  return 0;
};

const LinearProgressWithLabel = (props: LinearProgressProps & { value: number }) => {
  return (
    <Box sx={{ display: 'flex', alignItems: 'center' }}>
      <Box sx={{ width: '100%', mr: 1 }}>
        <LinearProgress variant="determinate" {...props} />
      </Box>
      <Box sx={{ minWidth: 35 }}>
        <Typography variant="body2" color="text.secondary">{`${Math.round(
          props.value,
        )}%`}</Typography>
      </Box>
    </Box>
  );
};

const setTaskDialogColor = (taskStatus: Status | undefined) => {
  if (!taskStatus) {
    return base.palette.background.default;
  }

  switch (taskStatus) {
    case Status.Failed:
      return base.palette.error.dark;

    case Status.Underway:
      return base.palette.success.dark;

    case Status.Queued:
      return base.palette.info.main;

    default:
      return base.palette.background.default;
  }
};

export interface TaskSummaryProps {
  stopShowing: () => void;
  task: TaskState | null;
  show: boolean;
}

export const TaskSummary = React.memo((props: TaskSummaryProps) => {
  const classes = useStyles();

  const { stopShowing, show, task } = props;

  const [openTaskDetailsLogs, setOpenTaskDetailsLogs] = React.useState(false);

  const getTaskMessage = (task: TaskState | null) => {
    if (!task || !task.phases || !task.active) {
      return "Can't get task message";
    }

    if (Object.values(task.phases)[task.active - 1]) {
      const message = Object.values(task.phases)[task.active - 1].detail;

      return message ? message : "Can't get task message";
    }
  };

  const returnDialogContent = () => {
    const contents = [
      {
        title: 'Task',
        value: task ? task.booking.id : '',
      },
      {
        title: 'Message',
        value: getTaskMessage(task),
      },
    ];

    return (
      <>
        {contents.map((message, index) => (
          <Grid key={index} container mb={1} alignItems="center" spacing={1}>
            <Grid item xs={3}>
              <Typography className={classes.subtitle}>{message.title}</Typography>
            </Grid>
            <Grid item xs={9}>
              <TextField
                size="small"
                value={message.value}
                multiline
                InputProps={{
                  readOnly: true,
                  className: classes.textField,
                }}
              />
            </Grid>
          </Grid>
        ))}
      </>
    );
  };

  return (
    <Dialog
      PaperProps={{
        style: {
          backgroundColor: setTaskDialogColor(task?.status),
          boxShadow: 'none',
        },
      }}
      open={show}
      onClose={stopShowing}
      maxWidth="xs"
    >
      <DialogTitle className={classes.title}>Info</DialogTitle>

      <Divider />
      <DialogTitle className={classes.title}>Task State</DialogTitle>
      <Box sx={{ width: '90%', ml: 3 }}>
        <LinearProgressWithLabel value={getTaskProgress(task)} />
      </Box>
      <DialogContent>{returnDialogContent()}</DialogContent>
      <DialogActions sx={{ justifyContent: 'center' }}>
        <Button
          size="small"
          variant="contained"
          onClick={() => setOpenTaskDetailsLogs(true)}
          autoFocus
        >
          Inspect
        </Button>
      </DialogActions>
      {openTaskDetailsLogs && (
        <TaskInspector
          task={task}
          open={openTaskDetailsLogs}
          onClose={() => setOpenTaskDetailsLogs(!openTaskDetailsLogs)}
        />
      )}
    </Dialog>
  );
});
