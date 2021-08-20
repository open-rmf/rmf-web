import { cleanup, render } from '@testing-library/react';
import React from 'react';
import { makeRobot } from '../robots/test-utils.spec';
import { DefaultMarker } from './default-marker';
import { RobotMarkerProps } from './robot-marker';

it('smoke test with different variants', () => {
  const variants: RobotMarkerProps['variant'][] = ['inConflict', 'normal', undefined];
  const robot = makeRobot();
  variants.forEach((v) => {
    render(
      <svg>
        <DefaultMarker
          color="blue"
          name={robot.name}
          model={robot.model}
          x={robot.location.x}
          y={robot.location.y}
          yaw={robot.location.yaw}
          fleetName="test_fleet"
          footprint={1}
          variant={v}
        />
      </svg>,
    );
    cleanup();
  });
});
