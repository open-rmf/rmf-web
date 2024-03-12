import React from 'react';
import {
  Box,
  LinearProgress,
  LinearProgressProps,
  Theme,
  Typography,
  Divider,
  TextField,
  useMediaQuery,
} from '@mui/material';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';
import { Status, TaskState } from 'api-client';
import { base, parseTaskRequestLabel, TaskRequestLabel } from 'react-components';
import { TaskInspector } from './task-inspector';
import { RmfAppContext } from '../rmf-app';
import { TaskCancelButton } from './task-cancellation';

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
    <Box component="div" sx={{ display: 'flex', alignItems: 'center' }}>
      <Box component="div" sx={{ width: '100%', mr: 1 }}>
        <LinearProgress variant="determinate" {...props} />
      </Box>
      <Box component="div" sx={{ minWidth: 35 }}>
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
  task?: TaskState;
}

export const TaskSummary = React.memo((props: TaskSummaryProps) => {
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const classes = useStyles();
  const rmf = React.useContext(RmfAppContext);

  const { onClose, task } = props;

  const [openTaskDetailsLogs, setOpenTaskDetailsLogs] = React.useState(false);
  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [label, setLabel] = React.useState<TaskRequestLabel>({});
  const [isOpen, setIsOpen] = React.useState(true);

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
    const sub = rmf.getTaskStateObs(task.booking.id).subscribe((subscribedTask) => {
      const requestLabel = parseTaskRequestLabel(subscribedTask);
      if (requestLabel) {
        setLabel(requestLabel);
      } else {
        setLabel({});
      }
      setTaskState(subscribedTask);
    });
    return () => sub.unsubscribe();
  }, [rmf, task]);

  const returnDialogContent = () => {
    const contents = [
      {
        title: 'ID',
        value: taskState ? taskState.booking.id : 'n/a.',
      },
      {
        title: 'Category',
        value: label.category ?? 'n/a',
      },
      {
        title: 'Pickup',
        value: label.pickup ?? 'n/a',
      },
      {
        title: 'Cart ID',
        value: label.cart_id ?? 'n/a',
      },
      {
        title: 'Dropoff',
        value: label.destination ?? 'n/a',
      },
      {
        title: 'Est. end time',
        value:
          taskState && taskState.unix_millis_finish_time
            ? `${new Date(taskState.unix_millis_finish_time).toLocaleTimeString()}`
            : 'n/a',
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
              sx={{
                '& .MuiFilledInput-root': {
                  fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                },
              }}
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
      open={isOpen}
      onClose={() => {
        setIsOpen(false);
        onClose();
      }}
      fullWidth
      maxWidth={isScreenHeightLessThan800 ? 'xs' : 'sm'}
    >
      <DialogTitle
        align="center"
        sx={{ fontSize: isScreenHeightLessThan800 ? '1.2rem' : '1.5rem' }}
      >
        Task Summary
      </DialogTitle>
      <Divider />
      {taskProgress && (
        <Box component="div" sx={{ width: '90%', ml: 3 }}>
          <LinearProgressWithLabel value={taskProgress * 100} />
        </Box>
      )}
      <DialogContent>{returnDialogContent()}</DialogContent>
      <DialogActions sx={{ justifyContent: 'center' }}>
        <TaskCancelButton
          taskId={taskState ? taskState.booking.id : null}
          size="small"
          variant="contained"
          color="secondary"
          sx={{
            fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
            padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
          }}
        />
      </DialogActions>
      {openTaskDetailsLogs && (
        <TaskInspector task={taskState} onClose={() => setOpenTaskDetailsLogs(false)} />
      )}
    </Dialog>
  );
});
