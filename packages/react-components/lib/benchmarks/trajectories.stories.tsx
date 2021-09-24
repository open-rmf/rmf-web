import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TrajectoryMarker } from '../map';
import { createRandomTrajectories } from '../map/test-utils.spec';

export default {
  title: 'Benchmarks/Trajectories',
  argTypes: {
    count: {
      name: 'Count',
    },
    conflict: {
      name: 'Conflict',
    },
  },
} as Meta;

export const TrajectoryMarkers: Story = ({ count, conflict, ...args }) => {
  const trajectories = React.useMemo(() => createRandomTrajectories(count), [count]);

  return (
    <svg viewBox="0 0 20 20" style={{ width: '100%', height: '100%', position: 'absolute' }}>
      {trajectories.map((traj) => (
        <TrajectoryMarker
          key={traj.id}
          color="green"
          conflict={conflict}
          trajectory={traj}
          loopAnimation
          animationScale={4}
          {...args}
        />
      ))}
    </svg>
  );
};
TrajectoryMarkers.args = {
  count: 100,
  conflict: false,
};
