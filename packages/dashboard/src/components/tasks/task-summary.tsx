import {
  Box,
  Divider,
  LinearProgress,
  LinearProgressProps,
  TextField,
  Typography,
  useMediaQuery,
  useTheme,
} from '@mui/material';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import {
  ApiServerModelsRmfApiTaskStateStatus as Status,
  TaskStateOutput as TaskState,
} from 'api-client';
import React from 'react';
import { base, getTaskBookingLabelFromTaskState, TaskBookingLabels } from 'react-components';

import { useRmfApi } from '../../hooks/use-rmf-api';
import { TaskCancelButton } from './task-cancellation';
import { TaskInspector } from './task-inspector';

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

const setTaskDialogColor = (taskStatus: Status | undefined | null) => {
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
  const rmfApi = useRmfApi();

  const { onClose, task } = props;

  const [openTaskDetailsLogs, setOpenTaskDetailsLogs] = React.useState(false);
  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [labels, setLabels] = React.useState<TaskBookingLabels | null>(null);
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
    if (!task) {
      return;
    }
    const sub = rmfApi.getTaskStateObs(task.booking.id).subscribe((subscribedTask) => {
      const taskBookingLabels = getTaskBookingLabelFromTaskState(subscribedTask);
      if (taskBookingLabels) {
        setLabels(taskBookingLabels);
      } else {
        setLabels(null);
      }
      setTaskState(subscribedTask);
    });
    return () => sub.unsubscribe();
  }, [rmfApi, task]);

  const theme = useTheme();

  const returnDialogContent = () => {
    const contents = [
      {
        title: 'ID',
        value: taskState ? taskState.booking.id : 'n/a.',
      },
      {
        title: 'Est. end time',
        value:
          taskState && taskState.unix_millis_finish_time
            ? `${new Date(taskState.unix_millis_finish_time).toLocaleTimeString()}`
            : 'n/a',
      },
    ];
    if (labels) {
      for (const key in labels) {
        contents.push({
          title: key,
          value: labels[key],
        });
      }
    }

    return (
      <>
        {contents.map((message, index) => (
          <div key={index}>
            <TextField
              label={message.title}
              id="standard-size-small"
              size="small"
              variant="filled"
              InputProps={{ readOnly: true }}
              fullWidth={true}
              multiline
              maxRows={4}
              margin="dense"
              value={message.value}
              sx={{
                '& .MuiFilledInput-root': {
                  fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                },
                background: theme.palette.background.default,
                '&:hover': {
                  backgroundColor: theme.palette.background.default,
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
