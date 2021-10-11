import { render } from '@testing-library/react';
import React from 'react';
import { NameLabel } from './label-marker';

describe('NameLabel', () => {
  it('smoke test', () => {
    const root = render(
      <svg>
        <NameLabel sourceX={100} sourceY={100} sourceRadius={0} text="test" />,
      </svg>,
    );
    expect(() => root.getByText('test')).not.toThrow();
  });
});
