import React from 'react';
import { describe, expect, it } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { RmfApi } from '../../services';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { LiftsTable } from './lifts-table';

describe('LiftsTable', () => {
  const Base = (props: React.PropsWithChildren<{}>) => {
    const rmfApi = React.useMemo<RmfApi>(() => {
      const mockRmfApi = new MockRmfApi();
      // mock out some api calls so they never resolves
      mockRmfApi.fleetsApi.getFleetsFleetsGet = () => new Promise(() => {});
      mockRmfApi.liftsApi.postLiftRequestLiftsLiftNameRequestPost = () => new Promise(() => {});
      return mockRmfApi;
    }, []);
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders with lifts table', () => {
    const root = render(
      <Base>
        <LiftsTable />
      </Base>,
    );
    expect(root.getByText('Name')).toBeTruthy();
    expect(root.getByText('Op. Mode')).toBeTruthy();
    expect(root.getByText('Current Floor')).toBeTruthy();
    expect(root.getByText('Destination Floor')).toBeTruthy();
    expect(root.getByText('Lift State')).toBeTruthy();
  });
});
