import React from 'react';
import { render } from '@testing-library/react';
import { CircleShape } from './circle-shape';
import { Euler, Vector3 } from 'three';
import { makeRobotData } from '../test-utils.spec';
import { Canvas } from '@react-three/fiber';

describe('CircleShape', () => {
  it('should render a circle and a line correctly', () => {
    const position = new Vector3(1, 1, 0);
    const rotation = new Euler(0, 0, Math.PI / 4);
    const onRobotClick = jasmine.createSpy();
    const robot = makeRobotData({
      name: 'test_robot_1',
      inConflict: false,
    });
    const segment = 32;

    const { container } = render(
      <Canvas>
        <CircleShape
          position={position}
          rotation={rotation}
          onRobotClick={onRobotClick}
          robot={robot}
          segment={segment}
        />
      </Canvas>,
    );

    const circle = container.querySelector('circle');
    expect(circle).toBeDefined();

    const line = container.querySelector('line');
    expect(line).toBeDefined();

    if (circle) {
      circle.dispatchEvent(new MouseEvent('click'));
      expect(onRobotClick).toHaveBeenCalled();
    }
  });
});
