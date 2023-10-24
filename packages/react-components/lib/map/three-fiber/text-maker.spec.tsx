import React from 'react';
import { render } from '@testing-library/react';
import { TextThreeRendering } from './text-maker';

describe('TextThreeRendering', () => {
  it('should handle the absence of text correctly', () => {
    const { container } = render(<TextThreeRendering position={[0, 0, 0]} />);
    const meshElements = container.querySelectorAll('mesh');
    expect(meshElements.length).toBe(2);
  });
});
