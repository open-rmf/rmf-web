import { render } from '@testing-library/react';
import React from 'react';
import { WaypointMarker } from '../../lib';

it('smoke test', () => {
  render(
    <svg>
      <WaypointMarker waypoint={{ name: 'test', params: [], x: 0, y: 0 }} />
    </svg>,
  );
});
