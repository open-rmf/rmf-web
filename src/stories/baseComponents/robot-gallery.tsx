import Grid from '@material-ui/core/Grid';
import Paper from '@material-ui/core/Paper';
import Typography from '@material-ui/core/Typography';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { ResourcesContext } from '../../components/app-contexts';
import ColorManager from '../../components/schedule-visualizer/colors';
import Robot, { RobotProps } from '../../components/schedule-visualizer/robot';
import { ResourceConfigurationsType } from '../../resource-manager';
import { makeRobot } from '../../mock/utils';

const colorManager = new ColorManager();

function makeRobotProps(
  robot: Partial<RomiCore.RobotState>,
  props?: Partial<Omit<RobotProps, 'robot'>>,
): RobotProps {
  props = props || {};
  return {
    robot: makeRobot(robot),
    colorManager: props.colorManager || colorManager,
    fleetName: props.fleetName || 'testFleet',
    footprint: props.footprint || 1,
    inConflict: props.inConflict,
    onClick: props.onClick,
  };
}

const robots: Record<string, RobotProps> = {
  Basic: makeRobotProps({ name: 'BasicRobot' }),
  'Really Really Loooonnnnnggggg Name': makeRobotProps({
    name: 'I have a really really loooonnnnnggggg name',
  }),
  'In Conflict': makeRobotProps({ name: 'ConflictingRobot' }, { inConflict: true }),
  'Name With Space': makeRobotProps({ name: 'I have spaces' }),
  'With Icon': makeRobotProps(
    { name: 'RobotWithIcon', model: 'fleetWithIcon' },
    { fleetName: 'fleetWithIcon' },
  ),
  'With Icon, In Conflict': makeRobotProps(
    { name: 'RobotWithIcon', model: 'fleetWithIcon' },
    { fleetName: 'fleetWithIcon', inConflict: true },
  ),
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

export default function RobotGallery(): React.ReactElement {
  return (
    <ResourcesContext.Provider value={resources}>
      <Grid container spacing={2}>
        {Object.keys(robots).map((k) => (
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
