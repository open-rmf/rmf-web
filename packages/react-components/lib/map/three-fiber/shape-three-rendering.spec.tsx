import React from 'react';
import { render, screen } from '@testing-library/react';
import { ShapeThreeRendering } from './shape-three-rendering';

describe('ShapeThreeRendering', () => {
  it('renders correctly with circle shape', () => {
    render(<ShapeThreeRendering position={[0, 0, 0]} color="red" circleShape={true} />);

    expect(screen.getByTestId('circle')).toBeTruthy();
  });

  it('renders correctly with non-circle shape', () => {
    render(
      <ShapeThreeRendering
        position={[0, 0, 0]}
        color="blue"
        text="Place one"
        circleShape={false}
      />,
    );

    expect(screen.getByTestId('non-circle')).toBeTruthy();
  });
});
