import React from 'react';
import {
  Box,
  Button,
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
    textField: {
      background: theme.palette.background.default,
      '&:hover': {
        backgroundColor: theme.palette.background.default,
      },
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
  onClose: () => void;
  task: TaskState | null;
  show: boolean;
}

export const TaskSummary = React.memo((props: TaskSummaryProps) => {
  const classes = useStyles();

  const { onClose, show, task } = props;

  const [openTaskDetailsLogs, setOpenTaskDetailsLogs] = React.useState(false);

  const taskProgress = React.useMemo(() => {
    if (
      !task ||
      !task.estimate_millis ||
      !task.unix_millis_start_time ||
      !task.unix_millis_finish_time
    ) {
      console.log(`Can't calculate task progress`);
      return undefined;
    }

    return Math.min(
      1.0 - task.estimate_millis / (task.unix_millis_finish_time - task.unix_millis_start_time),
      1,
    );
  }, [task]);

  const getTaskPhaseDetails = (task: TaskState | null) => {
    if (!task || !task.phases || !task.active) {
      return 'Failed to retrieve current task phase';
    }

    if (!Object.values(task.phases)[task.active - 1]) {
      return 'Failed to retrieve current task phase';
    }

    const message = Object.values(task.phases)[task.active - 1].detail;

    return message ? message : 'Failed to retrieve current task phase';
  };

  const returnDialogContent = () => {
    const contents = [
      {
        title: 'ID',
        value: task ? task.booking.id : '',
      },
      {
        title: 'Current phase',
        value: getTaskPhaseDetails(task),
      },
    ];

    return (
      <>
        {contents.map((message, index) => (
          <div key={index}>
            <TextField
              label={message.title}
              id="standard-size-small"
              size="small"
              variant="filled"
              InputProps={{ readOnly: true, className: classes.textField }}
              fullWidth={true}
              multiline
              maxRows={4}
              margin="dense"
              value={message.value}
            />
          </div>
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
      onClose={onClose}
      fullWidth
      maxWidth="sm"
    >
      <DialogTitle align="center">Task Summary</DialogTitle>
      <Divider />
      <DialogTitle align="center">Task State</DialogTitle>
      {taskProgress && (
        <Box sx={{ width: '90%', ml: 3 }}>
          <LinearProgressWithLabel value={taskProgress} />
        </Box>
      )}
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
