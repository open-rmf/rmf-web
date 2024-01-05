import { render } from '@testing-library/react';
import React from 'react';
import { makeTrajectory } from './test-utils.spec';
import { TrajectoryMarker } from './trajectory-marker';

it('smoke test', () => {
  render(
    <svg>
      <TrajectoryMarker trajectory={makeTrajectory()} color="green" conflict={false} />
      <TrajectoryMarker trajectory={makeTrajectory()} color="green" conflict={true} />
    </svg>,
  );
});
