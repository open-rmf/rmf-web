import { Grid, Paper, Typography } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { RobotMarker, RobotMarkerProps } from './robot-marker';

export default {
  title: 'Map/Robot Markers',
  component: RobotMarker,
  parameters: { controls: { include: ['onClick'], hideNoControlsWarning: true } },
} satisfies Meta;

function makeRobotMarkerProps(props?: Partial<Omit<RobotMarkerProps, 'robot'>>): RobotMarkerProps {
  props = props || {};
  return {
    cx: 0,
    cy: 0,
    r: 1,
    color: 'blue',
    inConflict: false,
    ...props,
  };
}

const robotMarkerProps: Record<string, RobotMarkerProps> = {
  Basic: makeRobotMarkerProps(),
  'In Conflict': makeRobotMarkerProps({ inConflict: true }),
  'With Icon': makeRobotMarkerProps({ iconPath: '/assets/tiny-robot.png' }),
  'With Icon, In Conflict': makeRobotMarkerProps({
    iconPath: '/assets/tiny-robot.png',
    inConflict: true,
  }),
};

export const Gallery: StoryFn = (args) => {
  return (
    <Grid container spacing={2}>
      {Object.keys(robotMarkerProps).map((k) => (
        <Grid item key={k}>
          <Paper>
            <Grid container direction="column" alignItems="center">
              <Grid item>
                <Typography align="center">{k}</Typography>
              </Grid>
              <Grid item>
                <svg viewBox="-4 -2 8 4" width={400} height={200}>
                  <RobotMarker {...robotMarkerProps[k]} {...args} />
                </svg>
              </Grid>
            </Grid>
          </Paper>
        </Grid>
      ))}
    </Grid>
  );
};
