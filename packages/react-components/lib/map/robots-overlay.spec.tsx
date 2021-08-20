import { render } from '@testing-library/react';
import React from 'react';
import { LMap } from './map';
import { RobotsOverlay } from './robots-overlay';
import { makeRobotData, officeL1Bounds } from './test-utils.spec';

describe('RobotsOverlay', () => {
  it('smoke test', () => {
    const root = render(
      <LMap bounds={officeL1Bounds}>
        <RobotsOverlay
          bounds={officeL1Bounds}
          robots={[makeRobotData({ fleet: 'test_fleet', name: 'test_robot' })]}
        />
      </LMap>,
    );
    expect(() => root.getByLabelText('test_robot')).not.toThrow();
  });
});
