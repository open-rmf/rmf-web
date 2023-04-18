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
import { RmfAppContext } from '../rmf-app';

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
  const rmf = React.useContext(RmfAppContext);

  const { onClose, show, task } = props;

  const [openTaskDetailsLogs, setOpenTaskDetailsLogs] = React.useState(false);
  const [taskState, setTaskState] = React.useState<TaskState | null>(null);

  const taskProgress = React.useMemo(() => {
    if (
      !taskState ||
      !taskState.estimate_millis ||
      !taskState.unix_millis_start_time ||
      !taskState.unix_millis_finish_time
    ) {
      console.log(`Can't calculate task progress`);
      return undefined;
    }

    return Math.min(
      1.0 -
        taskState.estimate_millis /
          (taskState.unix_millis_finish_time - taskState.unix_millis_start_time),
      1,
    );
  }, [taskState]);

  React.useEffect(() => {
    if (!rmf || !task) {
      return;
    }
    const sub = rmf
      .getTaskStateObs(task.booking.id)
      .subscribe((subscribedTask) => setTaskState(subscribedTask));
    return () => sub.unsubscribe();
  }, [rmf, task]);

  const getTaskPhaseDetails = (task: TaskState | null) => {
    if (!task || !task.phases || !task.active) {
      return 'Failed to retrieve current task phase';
    }

    if (!Object.values(task.phases)[task.active - 1]) {
      return 'Failed to retrieve current task phase';
    }

    const message = Object.values(task.phases)[task.active - 1].detail;

    if (message) {
      return message;
    }

    return `Failed to retrieve current task phase details of id ${task.booking.id} and category ${
      Object.values(task.phases)[task.active - 1].category
        ? Object.values(task.phases)[task.active - 1].category
        : ''
    }`;
  };

  const returnDialogContent = () => {
    const contents = [
      {
        title: 'ID',
        value: taskState ? taskState.booking.id : '',
      },
      {
        title: 'Current phase',
        value: getTaskPhaseDetails(taskState),
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
          backgroundColor: setTaskDialogColor(taskState?.status),
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
          <LinearProgressWithLabel value={taskProgress * 100} />
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
          task={taskState}
          open={openTaskDetailsLogs}
          onClose={() => setOpenTaskDetailsLogs(!openTaskDetailsLogs)}
        />
      )}
    </Dialog>
  );
});
