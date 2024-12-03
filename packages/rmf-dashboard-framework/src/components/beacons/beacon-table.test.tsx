import React from 'react';
import { describe, expect, it } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { RmfApi } from '../../services';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { BeaconsTable } from './beacons-table';

describe('BeaconsTable', () => {
  const Base = (props: React.PropsWithChildren<{}>) => {
    const rmfApi = React.useMemo<RmfApi>(() => {
      const mockRmfApi = new MockRmfApi();
      // mock out some api calls so they never resolves
      mockRmfApi.beaconsApi.getBeaconsBeaconsGet = () => new Promise(() => {});
      return mockRmfApi;
    }, []);
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders with beacons table', () => {
    const root = render(
      <Base>
        <BeaconsTable />
      </Base>,
    );
    expect(root.getByText('Name')).toBeTruthy();
    expect(root.getByText('Op. Mode')).toBeTruthy();
    expect(root.getByText('Level')).toBeTruthy();
    expect(root.getByText('Type')).toBeTruthy();
    expect(root.getByText('Beacon State')).toBeTruthy();
  });
});
