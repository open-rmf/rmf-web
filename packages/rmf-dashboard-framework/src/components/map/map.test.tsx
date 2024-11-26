import React from 'react';
import { describe, expect, it } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import Map from './map';

describe('Map', () => {
  const Base = (props: React.PropsWithChildren<{}>) => {
    const rmfApi = new MockRmfApi();
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders map without BuildingMap', () => {
    expect(() =>
      render(
        <Base>
          <Map
            defaultMapLevel="L1"
            defaultZoom={20}
            defaultRobotZoom={50}
            attributionPrefix="test_attribution_prefix"
          />
        </Base>,
      ),
    ).not.toThrow();
  });
});
