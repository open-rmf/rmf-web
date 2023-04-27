import {
  Box,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Divider,
  Grid,
  LinearProgress,
  LinearProgressProps,
  TextField,
  Theme,
  Typography,
} from '@mui/material';
import { makeStyles, createStyles } from '@mui/styles';
import React from 'react';
import { RmfAppContext } from '../rmf-app';
import { RobotTableData, base } from 'react-components';
import { RobotState, Status2, TaskState } from 'api-client';
import { EMPTY, combineLatest, mergeMap, of } from 'rxjs';
import { TaskInspector } from '../tasks/task-inspector';
import Battery80Icon from '@mui/icons-material/Battery80';

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

const setTaskDialogColor = (robotStatus: Status2 | undefined) => {
  if (!robotStatus) {
    return base.palette.background.default;
  }

  switch (robotStatus) {
    case Status2.Error:
      return base.palette.error.dark;

    case Status2.Working:
      return base.palette.success.dark;

    default:
      return base.palette.warning.main;
  }
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

interface RobotSummaryProps {
  onClose: () => void;
  robot: RobotTableData;
}

export const RobotSummary = React.memo(({ onClose, robot }: RobotSummaryProps) => {
  const classes = useStyles();
  const rmf = React.useContext(RmfAppContext);

  const [isOpen, setIsOpen] = React.useState(true);
  const [robotState, setRobotState] = React.useState<RobotState | null>(null);
  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [openTaskDetailsLogs, setOpenTaskDetailsLogs] = React.useState(false);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf
      .getFleetStateObs(robot.fleet)
      .pipe(
        mergeMap((fleetState) => {
          const robotState = fleetState?.robots?.[robot.name];
          const taskObs = robotState?.task_id ? rmf.getTaskStateObs(robotState.task_id) : of(null);
          return robotState ? combineLatest([of(robotState), taskObs]) : EMPTY;
        }),
      )
      .subscribe(([robotState, taskState]) => {
        setRobotState(robotState);
        setTaskState(taskState);
      });
    return () => sub.unsubscribe();
  }, [rmf, robot.fleet, robot.name]);

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

  const returnDialogContent = () => {
    const contents = [
      {
        title: 'Assigned Tasks',
        value: taskState ? taskState.booking.id : 'No task',
      },
      {
        title: 'Location',
        value: robotState ? robotState.location?.map : '',
      },
      {
        title: 'Est. End Time',
        value: taskState?.unix_millis_finish_time
          ? `${new Date(taskState?.unix_millis_finish_time).toLocaleString()}`
          : '-',
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
          backgroundColor: setTaskDialogColor(robotState?.status),
          boxShadow: 'none',
        },
      }}
      open={isOpen}
      onClose={() => {
        setIsOpen(false);
        onClose();
      }}
      fullWidth
      maxWidth="sm"
    >
      <Grid container mb={1} alignItems="center" spacing={1}>
        <Grid item xs={10}>
          <DialogTitle align="center">Robot Summary: {robotState?.name}</DialogTitle>
        </Grid>
        <Grid item xs={2}>
          <Grid container justifyContent="flex-end">
            <Typography variant="subtitle1">{`${
              robotState?.battery ? robotState?.battery * 100 : 0
            }%`}</Typography>
            <Battery80Icon />
          </Grid>
        </Grid>
      </Grid>
      <Divider />
      {taskProgress && (
        <>
          <Typography variant="h6" fontWeight="bold" align="center">
            Task Progress
          </Typography>
          <Box sx={{ width: '90%', ml: 3 }}>
            <LinearProgressWithLabel value={taskProgress * 100} />
          </Box>
        </>
      )}
      <DialogContent>{returnDialogContent()}</DialogContent>
      <DialogActions sx={{ justifyContent: 'center' }}>
        <Button
          size="small"
          variant="contained"
          onClick={() => setOpenTaskDetailsLogs(true)}
          autoFocus
          disabled={taskState === null}
        >
          Inspect Task
        </Button>
      </DialogActions>
      {openTaskDetailsLogs && taskState && (
        <TaskInspector task={taskState} onClose={() => setOpenTaskDetailsLogs(false)} />
      )}
    </Dialog>
  );
});
