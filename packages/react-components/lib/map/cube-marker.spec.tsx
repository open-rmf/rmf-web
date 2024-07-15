import React from 'react';
import { render } from '@testing-library/react';
import { CubeMaker } from './cube-maker';
import { Euler } from 'three';

describe('CubeMaker', () => {
  it('should render a cube with the provided properties.', () => {
    const position = [0, 0, 0];
    const size = [1, 1, 1];
    const rot = new Euler(0, 0, 0);
    const color = 'red';

    const { container } = render(
      <CubeMaker position={position} size={size} rot={rot} color={color} />,
    );

    const cubeElement = container.querySelector('mesh');

    expect(cubeElement).toBeTruthy();

    expect(cubeElement?.getAttribute('position')).toBe('0,0,0');
    expect(cubeElement?.getAttribute('scale')).toBe('1,1,1');

    const materialElement = cubeElement?.querySelector('meshStandardMaterial');
    expect(materialElement?.getAttribute('color')).toBe('red');
  });
});
