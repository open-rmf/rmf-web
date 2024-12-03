import { render, waitFor } from '@testing-library/react';
import React, { act } from 'react';
import { describe, expect, it } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, TestProviders } from '../../utils/test-utils.test';
import Map from './map';
import { officeMap } from './test-utils.test';

// Obtained from https://github.com/ZeeCoder/use-resize-observer/issues/40
// to resolve ResizeObserver related errors during testing.
class ResizeObserver {
  observe() {}
  unobserve() {}
  disconnect() {}
}

describe('Map', () => {
  window.ResizeObserver = ResizeObserver;
  const rmfApi = new MockRmfApi();
  rmfApi.buildingApi.getBuildingMapBuildingMapGet = () => new Promise(() => {});
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders map with office BuildingMap', async () => {
    const root = render(
      <Base>
        <Map
          defaultMapLevel="L1"
          defaultZoom={20}
          defaultRobotZoom={50}
          attributionPrefix="test_attribution_prefix"
        />
      </Base>,
    );

    act(() => {
      rmfApi.buildingMapObs.next(officeMap);
    });

    await expect(waitFor(() => root.getByText('test_attribution_prefix'))).resolves.toBeTruthy();
  });
});
