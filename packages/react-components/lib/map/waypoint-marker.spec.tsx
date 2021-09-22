import { render } from '@testing-library/react';
import React from 'react';
import { WaypointMarker } from './waypoint-marker';

describe('WaypointMarker', () => {
  it('smoke test', () => {
    render(
      <svg>
        <WaypointMarker />
      </svg>,
    );
  });
});
