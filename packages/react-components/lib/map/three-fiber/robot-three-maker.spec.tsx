import React from 'react';
import { RobotData, RobotThreeMaker } from './robot-three-maker';
import { Euler, Vector3 } from 'three';
import ReactThreeTestRenderer from '@react-three/test-renderer';

describe('RobotThreeMaker', () => {
  it('renders robot properly', async () => {
    const robot: RobotData = {
      fleet: 'Fleet 1',
      name: 'Robot 1',
      model: 'Model 1',
      footprint: 1.0,
      color: 'blue',
      scale: 1.0,
    };

    const renderer = await ReactThreeTestRenderer.create(
      <RobotThreeMaker
        robot={robot}
        position={new Vector3(0, 0, 0)}
        rotation={new Euler(0, 0, 0)}
        circleSegment={64}
        robotLabel={true}
      />,
    );

    const mesh = renderer.scene.children[0].allChildren;

    expect(mesh.length).toBe(1);
  });
});
