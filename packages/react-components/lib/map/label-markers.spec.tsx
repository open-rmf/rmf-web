import { render } from '@testing-library/react';
import React from 'react';
import { NameLabel } from './label-marker';

describe('NameLabel', () => {
  it('smoke test', () => {
    const root = render(
      <svg>
        <NameLabel anchorX={0} anchorY={0} arrowLength={1} text="test" fontSize={1} />,
      </svg>,
    );
    expect(() => root.getByText('test')).not.toThrow();
  });
});
