import { Grid, Paper, Typography } from '@material-ui/core';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { RobotMarker, RobotMarkerProps } from '../lib';
import { makeRobot } from '../tests/robots/test-utils';

export default {
  title: 'Robot Markers',
  component: RobotMarker,
} as Meta;

function makeRobotProps(
  robot?: Partial<RmfModels.RobotState>,
  props?: Partial<Omit<RobotMarkerProps, 'robot'>>,
): RobotMarkerProps {
  props = props || {};
  return {
    robot: makeRobot(robot),
    fleetName: 'testFleet',
    footprint: 1,
    ...props,
  };
}

const robots: Record<string, RobotMarkerProps> = {
  Basic: makeRobotProps({ name: 'BasicRobot' }),
  'Really Really Loooonnnnnggggg Name': makeRobotProps({
    name: 'I have a really really loooonnnnnggggg name',
  }),
  'In Conflict': makeRobotProps({ name: 'ConflictingRobot' }, { variant: 'inConflict' }),
  'Name With Space': makeRobotProps({ name: 'I have spaces' }),
  'With Icon': makeRobotProps(
    { name: 'RobotWithIcon', model: 'fleetWithIcon' },
    { fleetName: 'fleetWithIcon', iconPath: '/assets/tinyRobot.png' },
  ),
  'With Icon, In Conflict': makeRobotProps(
    { name: 'RobotWithIcon', model: 'fleetWithIcon' },
    { fleetName: 'fleetWithIcon', iconPath: '/assets/tinyRobot.png', variant: 'inConflict' },
  ),
};

export const Gallery: Story = (args) => {
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

export const NoTranslate: Story = (args) => {
  return (
    <svg viewBox="-2 -2 4 4" width={400} height={400}>
      <RobotMarker
        robot={makeRobot({
          location: {
            level_name: 'test',
            x: 10,
            y: 10,
            yaw: 0,
            t: { sec: 0, nanosec: 0 },
            index: 0,
          },
        })}
        fleetName="test_fleet"
        footprint={1}
        translate={false}
        {...args}
      />
    </svg>
  );
};
