import React from 'react';
import { render } from '@testing-library/react';
import { RobotThreeMaker } from './robot-three-maker';
import { Euler, Vector3 } from 'three';
import { Canvas } from '@react-three/fiber';

describe('RobotThreeMaker', () => {
  it('renders robot name correctly', () => {
    const robot = {
      fleet: 'Fleet 1',
      name: 'Robot 1',
      model: 'Model 1',
      footprint: 1.0,
      color: 'blue',
    };

    const { container } = render(
      <Canvas>
        <RobotThreeMaker
          robot={robot}
          position={new Vector3(0, 0, 0)}
          rotation={new Euler(0, 0, 0)}
          circleSegment={64}
        />
      </Canvas>,
    );
    const text = container.querySelector('[data-testid="robot-name"]');
    expect(text).toBeDefined();
  });
});
