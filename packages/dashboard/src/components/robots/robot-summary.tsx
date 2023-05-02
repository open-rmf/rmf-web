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
import BatteryFullIcon from '@mui/icons-material/BatteryFull';
import Battery60Icon from '@mui/icons-material/Battery60';
import Battery20Icon from '@mui/icons-material/Battery20';
import BatteryChargingFullIcon from '@mui/icons-material/BatteryChargingFull';

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

const showBatteryIcon = (robot: RobotState, robotBattery: number) => {
  if (robot.status === Status2.Charging) {
    return <BatteryChargingFullIcon />;
  }

  const batteryIcons: Record<string, JSX.Element> = {
    '100': <BatteryFullIcon />,
    '80': <Battery60Icon />,
    '40': <Battery20Icon />,
    '0': <Battery20Icon />,
  };

  const key = Object.keys(batteryIcons).find((level) => robotBattery <= parseInt(level));

  return key ? batteryIcons[key] : null;
};

export const RobotSummary = React.memo(({ onClose, robot }: RobotSummaryProps) => {
  const classes = useStyles();
  const rmf = React.useContext(RmfAppContext);

  const [isOpen, setIsOpen] = React.useState(true);
  const [robotState, setRobotState] = React.useState<RobotState | null>(null);
  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [openTaskDetailsLogs, setOpenTaskDetailsLogs] = React.useState(false);
  const [location, setLocation] = React.useState('');
  const [destination, setDestination] = React.useState('');

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

  React.useEffect(() => {
    if (!taskState || !taskState.phases || !taskState.active) {
      setLocation('Failed to retrieve current location');
      setDestination('Failed to retrieve robot destination');
      return;
    }

    const message = Object.values(taskState.phases)[taskState.active - 1]?.detail;

    if (message) {
      const regex = /\[place:(.*?)\]/g;

      let match;
      const waypoints = [];

      // Iterate over all matches found by the regular expression
      while ((match = regex.exec(message.toString()))) {
        waypoints.push(match[1]);
      }
      setLocation(waypoints[0]);
      setDestination(waypoints[1]);
    }
  }, [taskState]);

  const returnDialogContent = () => {
    const contents = [
      {
        title: 'Assigned Tasks',
        value: taskState ? taskState.booking.id : 'No task',
      },
      {
        title: 'Est. End Time',
        value: taskState?.unix_millis_finish_time
          ? `${new Date(taskState?.unix_millis_finish_time).toLocaleString()}`
          : '-',
      },
    ];

    if (taskState) {
      contents.push(
        {
          title: 'Navigation start',
          value: location,
        },
        {
          title: 'Navigation destination',
          value: destination,
        },
      );
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
        <Grid item xs={2}></Grid>
        <Grid item xs={8}>
          <DialogTitle align="center">Robot Summary: {robotState?.name}</DialogTitle>
        </Grid>
        <Grid item xs={2}>
          <Grid container justifyContent="flex-end">
            <Typography variant="subtitle1">{`${
              robotState?.battery ? robotState?.battery * 100 : 0
            }%`}</Typography>
            {robotState && (
              <>{showBatteryIcon(robot, robotState.battery ? robotState?.battery * 100 : 0)}</>
            )}
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
