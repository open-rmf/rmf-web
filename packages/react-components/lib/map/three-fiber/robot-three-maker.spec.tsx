import React from 'react';
import { RobotThreeMaker } from './robot-three-maker';
import { Euler, Vector3 } from 'three';
import ReactThreeTestRenderer from '@react-three/test-renderer';

describe('RobotThreeMaker', () => {
  it('renders color properly', async () => {
    const robot = {
      fleet: 'Fleet 1',
      name: 'Robot 1',
      model: 'Model 1',
      footprint: 1.0,
      color: 'blue',
    };

    const renderer = await ReactThreeTestRenderer.create(
      <RobotThreeMaker
        robot={robot}
        position={new Vector3(0, 0, 0)}
        rotation={new Euler(0, 0, 0)}
        circleSegment={64}
      />,
    );

    const mesh = renderer.scene.children[0].allChildren;

    expect(mesh.length).toBe(1);
  });
});
