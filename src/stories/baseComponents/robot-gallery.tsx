import Grid from '@material-ui/core/Grid';
import Paper from '@material-ui/core/Paper';
import Typography from '@material-ui/core/Typography';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { ResourcesContext } from '../../app-contexts';
import ColorManager from '../../components/schedule-visualizer/colors';
import Robot, { RobotProps } from '../../components/schedule-visualizer/robot';
import { ResourceConfigurationsType } from '../../resource-manager';

const baseRobot: RomiCore.RobotState = {
  name: '',
  battery_percent: 100,
  location: { level_name: '', x: 0, y: 0, t: { sec: 0, nanosec: 0 }, yaw: 0 },
  mode: { mode: RomiCore.RobotMode.MODE_IDLE },
  model: 'BaseModel',
  path: [],
  task_id: '',
};

const colorManager = new ColorManager();

const baseRobotProps: RobotProps = {
  robot: baseRobot,
  colorManager: colorManager,
  fleetName: '',
  footprint: 1,
  inConflict: false,
};

const robots: Record<string, RobotProps> = {
  Basic: {
    ...baseRobotProps,
    robot: {
      ...baseRobot,
      name: 'BasicRobot',
    },
  },
  'Really Really Loooonnnnnggggg Name': {
    ...baseRobotProps,
    robot: {
      ...baseRobot,
      name: 'I have a really really loooonnnnnggggg name',
    },
  },
  'In Conflict': {
    ...baseRobotProps,
    robot: {
      ...baseRobot,
      name: 'ConflictingRobot',
    },
    inConflict: true,
  },
  'Name With Space': {
    ...baseRobotProps,
    robot: {
      ...baseRobot,
      name: 'I have spaces',
    },
  },
  'With Icon': {
    ...baseRobotProps,
    robot: {
      ...baseRobot,
      name: 'RobotWithIcon',
      model: 'fleetWithIcon',
    },
    fleetName: 'fleetWithIcon',
  },
};

const resources: ResourceConfigurationsType = {
  robots: {
    fleetWithIcon: {
      icons: {
        fleetWithIcon: '/fleetWithIcon.png',
      },
    },
  },
};

// const useStyles = makeStyles(() => ({
//   spin: {
//     animation: '$spin 2s linear infinite',
//   },
//   '@keyframes spin': {
//     '100%': {
//       transform: 'rotate(360deg);',
//     },
//   },
// }));

export default function RobotGallery(): React.ReactElement {
  return (
    <ResourcesContext.Provider value={resources}>
      <Grid container spacing={2}>
        {Object.keys(robots).map(k => (
          <Grid item key={k}>
            <Paper>
              <Grid container direction="column" alignItems="center">
                <Grid item>
                  <Typography align="center">{k}</Typography>
                </Grid>
                <Grid item>
                  <svg viewBox="-2 -2 4 4" style={{ minWidth: 200 }}>
                    <Robot {...robots[k]} />
                  </svg>
                </Grid>
              </Grid>
            </Paper>
          </Grid>
        ))}
      </Grid>
    </ResourcesContext.Provider>
  );
}
