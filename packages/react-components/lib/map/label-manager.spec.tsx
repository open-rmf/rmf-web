import { render } from '@testing-library/react';
import React from 'react';
import { ManagedNameLabel } from './label-manager';
import { LMap } from './map';
import { SVGOverlay } from './svg-overlay';

describe('ManagedNameLabel', () => {
  it('smoke test', () => {
    const root = render(
      <LMap
        bounds={[
          [0, 0],
          [1, 1],
        ]}
      >
        <SVGOverlay
          bounds={[
            [0, 0],
            [1, 1],
          ]}
        >
          <ManagedNameLabel labelTarget={{ centerX: 0, centerY: 0, radius: 0 }} text="test" />
        </SVGOverlay>
      </LMap>,
    );
    expect(() => root.getByText('test')).not.toThrow();
  });
});
