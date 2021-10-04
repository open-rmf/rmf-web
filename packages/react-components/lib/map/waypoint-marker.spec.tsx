import { render } from '@testing-library/react';
import React from 'react';
import { WaypointMarker } from './waypoint-marker';

describe('WaypointMarker', () => {
  it('smoke test', () => {
    render(
      <svg>
        <WaypointMarker cx={0} cy={0} size={1} />
      </svg>,
    );
  });
});
