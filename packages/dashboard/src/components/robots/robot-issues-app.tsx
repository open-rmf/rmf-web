import { Box, CardContent, Typography } from '@mui/material';
import { RobotState, TaskState } from 'api-client';
import React from 'react';
import { RobotIssues } from 'react-components';
import { combineLatest, EMPTY, mergeMap, of, switchMap } from 'rxjs';
import { AppEvents } from '../app-events';
import { createMicroApp } from '../micro-app';
import { RmfAppContext } from '../rmf-app';

export const RobotIssuesApp = createMicroApp('Robot Issues', () => {
  const rmf = React.useContext(RmfAppContext);

  const [robotState, setRobotState] = React.useState<RobotState | null>(null);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = AppEvents.robotSelect.subscribe((data) => {
      if (!data) {
        return of([null, null]);
      }
      const [fleet, name] = data;
      return rmf
        .getFleetStateObs(fleet)
        .pipe(
          mergeMap((state) =>
            state.robots && state.robots[name] ? of(state.robots[name]) : EMPTY,
          ),
        )
        .subscribe(setRobotState);
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  return (
    <CardContent sx={{ height: '100%', boxSizing: 'border-box' }}>
      {robotState ? (
        <RobotIssues robotIssues={robotState.issues} />
      ) : (
        <Box
          sx={{ display: 'flex', height: '100%', alignItems: 'center', justifyContent: 'center' }}
        >
          <Typography variant="h6" align="center">
            Click on a robot to view its issues.
          </Typography>
        </Box>
      )}
    </CardContent>
  );
});
