import { render } from '@testing-library/react';
import React from 'react';
import { LMap } from './map';
import { makePlace, makeVertex, officeL1Bounds } from './test-utils.spec';
import { WaypointsOverlay } from './waypoints-overlay';

describe('WaypointsOverlay', () => {
  it('smoke test', () => {
    const root = render(
      <LMap bounds={officeL1Bounds}>
        <WaypointsOverlay
          bounds={officeL1Bounds}
          waypoints={[makePlace({ vertex: makeVertex({ name: 'test_place' }) })]}
        />
      </LMap>,
    );
    expect(() => root.getByLabelText('test_place')).not.toThrow();
  });
});
