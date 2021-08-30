import { render } from '@testing-library/react';
import React from 'react';
import { LMap } from './map';
import { makeTrajectory, makeTrajectoryData, officeL1Bounds } from './test-utils.spec';
import { TrajectoriesOverlay } from './trajectories-overlay';

describe('TrajectoriesOverlay', () => {
  it('smoke test', async () => {
    const root = render(
      <LMap bounds={officeL1Bounds}>
        <TrajectoriesOverlay
          bounds={officeL1Bounds}
          trajectoriesData={[makeTrajectoryData({ trajectory: makeTrajectory({ id: 0 }) })]}
        />
      </LMap>,
    );
    expect(() => root.getByLabelText('trajectory 0')).not.toThrow();
  });
});
