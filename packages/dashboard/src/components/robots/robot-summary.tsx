import {
  Battery0Bar,
  Battery1Bar,
  Battery2Bar,
  Battery3Bar,
  Battery4Bar,
  Battery5Bar,
  Battery6Bar,
  BatteryChargingFull,
  BatteryFull,
  BatteryUnknown,
} from '@mui/icons-material';
import {
  Box,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Divider,
  Grid,
  LinearProgress,
  LinearProgressProps,
  TextField,
  Typography,
  useMediaQuery,
  useTheme,
} from '@mui/material';
import {
  ApiServerModelsRmfApiRobotStateStatus as Status,
  RobotState,
  TaskStateOutput as TaskState,
} from 'api-client';
import React from 'react';
import { base, RobotTableData } from 'react-components';
import { combineLatest, EMPTY, mergeMap, of } from 'rxjs';

import { useRmfApi } from '../../hooks/use-rmf-api';
import { TaskCancelButton } from '../tasks/task-cancellation';
import { TaskInspector } from '../tasks/task-inspector';
import { RobotDecommissionButton } from './robot-decommission';

const setTaskDialogColor = (robotStatus: Status | undefined | null) => {
  if (!robotStatus) {
    return base.palette.background.default;
  }

  switch (robotStatus) {
    case Status.Error:
      return base.palette.error.dark;

    case Status.Working:
      return base.palette.success.dark;

    default:
      return base.palette.warning.main;
  }
};

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

interface RobotSummaryProps {
  onClose: () => void;
  robot: RobotTableData;
}

const showBatteryIcon = (robot: RobotState, robotBattery: number) => {
  if (robot.status === Status.Charging) {
    return <BatteryChargingFull />;
  }

  const batteryIcons: Record<number, JSX.Element> = {
    0: <Battery0Bar />,
    16: <Battery1Bar />,
    32: <Battery2Bar />,
    48: <Battery3Bar />,
    64: <Battery4Bar />,
    80: <Battery5Bar />,
    96: <Battery6Bar />,
    100: <BatteryFull />,
  };

  for (const level in batteryIcons) {
    if (robotBattery >= parseInt(level)) {
      continue;
    } else {
      return batteryIcons[level];
    }
  }
  return <BatteryUnknown />;
};

export const RobotSummary = React.memo(({ onClose, robot }: RobotSummaryProps) => {
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const rmfApi = useRmfApi();

  const [isOpen, setIsOpen] = React.useState(true);
  const [robotState, setRobotState] = React.useState<RobotState | null>(null);
  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [openTaskDetailsLogs, setOpenTaskDetailsLogs] = React.useState(false);
  const [navigationStart, setNavigationStart] = React.useState<string | null>(null);
  const [navigationDestination, setNavigationDestination] = React.useState<string | null>(null);

  React.useEffect(() => {
    const sub = rmfApi
      .getFleetStateObs(robot.fleet)
      .pipe(
        mergeMap((fleetState) => {
          const robotState = fleetState?.robots?.[robot.name];
          const taskObs = robotState?.task_id
            ? rmfApi.getTaskStateObs(robotState.task_id)
            : of(null);
          return robotState ? combineLatest([of(robotState), taskObs]) : EMPTY;
        }),
      )
      .subscribe(([robotState, taskState]) => {
        setRobotState(robotState);
        setTaskState(taskState);
      });
    return () => sub.unsubscribe();
  }, [rmfApi, robot.fleet, robot.name]);

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
      setNavigationStart(null);
      setNavigationDestination(null);
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

      setNavigationStart(waypoints[0]);
      setNavigationDestination(waypoints[1]);
    } else {
      setNavigationStart('-');
      setNavigationDestination('-');
      console.error("Failed to retrieve robot's current navigation start and destination.");
    }
  }, [taskState]);

  const theme = useTheme();

  const returnDialogContent = () => {
    const contents = [
      {
        title: 'Assigned tasks',
        value: taskState ? taskState.booking.id : 'No task',
      },
      {
        title: 'Est. end time',
        value: taskState?.unix_millis_finish_time
          ? `${new Date(taskState?.unix_millis_finish_time).toLocaleString()}`
          : '-',
      },
    ];

    if (taskState) {
      contents.push(
        {
          title: 'Navigation start',
          value: navigationStart ? navigationStart : '-',
        },
        {
          title: 'Navigation destination',
          value: navigationDestination ? navigationDestination : '-',
        },
      );
    }

    if (robotState && robotState.commission) {
      const commission = robotState.commission;
      contents.push({
        title: `[${
          commission.dispatch_tasks === false ? 'Decommissioned' : 'Commissioned'
        }] status`,
        value:
          `Direct tasks  : ${commission.direct_tasks ?? 'n/a'}\n` +
          `Dispatch tasks: ${commission.dispatch_tasks ?? 'n/a'}\n` +
          `Idle Behavior : ${commission.idle_behavior ?? 'n/a'}`,
      });
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
      maxWidth={isScreenHeightLessThan800 ? 'xs' : 'sm'}
    >
      <Grid container mb={1} alignItems="center" spacing={1}>
        <Grid item xs={2}></Grid>
        <Grid item xs={8}>
          <DialogTitle
            align="center"
            sx={{ fontSize: isScreenHeightLessThan800 ? '1.2rem' : '1.5rem' }}
          >
            Robot summary: {robotState?.name}
          </DialogTitle>
        </Grid>
        <Grid item xs={2}>
          <Grid container justifyContent="flex-end">
            <Typography variant="subtitle1">{`${
              robotState?.battery ? (robotState.battery * 100).toFixed(0) : 0
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
          <Typography variant="body2" fontWeight="bold" ml={3} mt={1}>
            Task progress
          </Typography>
          <Box component="div" sx={{ width: '95%', ml: 3 }}>
            <LinearProgressWithLabel value={taskProgress * 100} />
          </Box>
        </>
      )}
      <DialogContent>{returnDialogContent()}</DialogContent>
      <DialogActions sx={{ justifyContent: 'center' }}>
        <RobotDecommissionButton
          fleet={robot.fleet}
          robotState={robotState}
          size="small"
          variant="contained"
          color="secondary"
          sx={{
            fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
            padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
          }}
        />
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
      {openTaskDetailsLogs && taskState && (
        <TaskInspector task={taskState} onClose={() => setOpenTaskDetailsLogs(false)} />
      )}
    </Dialog>
  );
});
