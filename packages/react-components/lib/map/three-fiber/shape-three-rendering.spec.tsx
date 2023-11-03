import React from 'react';
import { ShapeThreeRendering } from './shape-three-rendering';
import ReactThreeTestRenderer from '@react-three/test-renderer';

describe('ShapeThreeRendering', () => {
  it('renders correctly with non-circle shape', async () => {
    const renderer = await ReactThreeTestRenderer.create(
      <ShapeThreeRendering
        position={[0, 0, 0]}
        color="blue"
        text="Place one"
        circleShape={false}
      />,
    );

    const mesh = renderer.scene.children[0].allChildren;

    expect(mesh.length).toBe(1);
  });

  it('renders color properly', async () => {
    const renderer = await ReactThreeTestRenderer.create(
      <ShapeThreeRendering position={[0, 0, 0]} color="blue" text="Place one" circleShape={true} />,
    );

    const searchByColor = renderer.scene.findAll((node) => node.props.color === 'blue');

    expect(searchByColor.length).toBe(1);
  });
});
