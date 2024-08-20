import { Box, CardContent, Typography } from '@mui/material';
import { RobotState, TaskStateOutput } from 'api-client';
import React from 'react';
import { RobotInfo } from 'react-components';
import { combineLatest, EMPTY, mergeMap, of, switchMap, throttleTime } from 'rxjs';

import { useRmfApi } from '../../hooks/use-rmf-api';
import { AppEvents } from '../app-events';

export const RobotInfoCard = () => {
  const rmfApi = useRmfApi();

  const [robotState, setRobotState] = React.useState<RobotState | null>(null);
  const [taskState, setTaskState] = React.useState<TaskStateOutput | null>(null);
  React.useEffect(() => {
    const sub = AppEvents.robotSelect
      .pipe(
        switchMap((data) => {
          if (!data) {
            return of([null, null]);
          }
          const [fleet, name] = data;
          return rmfApi.getFleetStateObs(fleet).pipe(
            throttleTime(3000, undefined, { leading: true, trailing: true }),
            mergeMap((fleetState) => {
              const robotState = fleetState?.robots?.[name];
              const taskObs = robotState?.task_id
                ? rmfApi.getTaskStateObs(robotState.task_id)
                : of(null);
              return robotState ? combineLatest([of(robotState), taskObs]) : EMPTY;
            }),
          );
        }),
      )
      .subscribe(([robotState, taskState]) => {
        setRobotState(robotState);
        setTaskState(taskState);
      });
    return () => sub.unsubscribe();
  }, [rmfApi]);

  const taskProgress = React.useMemo(() => {
    if (
      !taskState ||
      !taskState.estimate_millis ||
      !taskState.unix_millis_start_time ||
      !taskState.unix_millis_finish_time
    ) {
      return undefined;
    }
    return Math.min(
      1.0 -
        taskState.estimate_millis /
          (taskState.unix_millis_finish_time - taskState.unix_millis_start_time),
      1,
    );
  }, [taskState]);

  return (
    <CardContent sx={{ height: '100%', boxSizing: 'border-box' }}>
      {robotState ? (
        <RobotInfo
          robotName={robotState.name || 'unknown'}
          assignedTask={robotState.task_id || undefined}
          battery={robotState.battery != null ? +robotState.battery.toFixed(2) : undefined}
          estFinishTime={
            taskState?.unix_millis_finish_time != null
              ? taskState.unix_millis_finish_time
              : undefined
          }
          taskProgress={taskProgress}
          taskStatus={taskState?.status}
        />
      ) : (
        <Box
          component="div"
          sx={{ display: 'flex', height: '100%', alignItems: 'center', justifyContent: 'center' }}
        >
          <Typography variant="h6" align="center">
            Click on a robot to view more information
          </Typography>
        </Box>
      )}
    </CardContent>
  );
};

export default RobotInfoCard;
