import { Grid, Paper, Typography } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { RobotMarker, RobotMarkerProps } from '../lib';
import { makeRobot } from '../tests/robots/test-utils';

export default {
  title: 'Robot Markers',
  component: RobotMarker,
} as Meta;

function makeRobotProps(
  robot?: Partial<RomiCore.RobotState>,
  props?: Partial<Omit<RobotMarkerProps, 'robot'>>,
): RobotMarkerProps {
  props = props || {};
  return {
    robot: makeRobot(robot),
    fleetName: props.fleetName || 'testFleet',
    footprint: props.footprint || 1,
    inConflict: props.inConflict,
    onClick: props.onClick,
  };
}

const robots: Record<string, RobotMarkerProps> = {
  Basic: makeRobotProps({ name: 'BasicRobot' }),
  'Really Really Loooonnnnnggggg Name': makeRobotProps({
    name: 'I have a really really loooonnnnnggggg name',
  }),
  'In Conflict': makeRobotProps({ name: 'ConflictingRobot' }, { inConflict: true }),
  'Name With Space': makeRobotProps({ name: 'I have spaces' }),
  'With Icon': makeRobotProps(
    { name: 'RobotWithIcon', model: 'fleetWithIcon' },
    { fleetName: 'fleetWithIcon', iconPath: '/resources/tinyRobot.png' },
  ),
  'With Icon, In Conflict': makeRobotProps(
    { name: 'RobotWithIcon', model: 'fleetWithIcon' },
    { fleetName: 'fleetWithIcon', iconPath: '/resources/tinyRobot.png', inConflict: true },
  ),
};

export const RobotGallery: Story = (args) => {
  return (
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
                  <RobotMarker {...robots[k]} {...args} />
                </svg>
              </Grid>
            </Grid>
          </Paper>
        </Grid>
      ))}
    </Grid>
  );
};
