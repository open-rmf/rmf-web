import { BuildingMap, LiftState } from 'api-client';
import React, { act } from 'react';
import { LiftState as RmfLiftState } from 'rmf-models/ros/rmf_lift_msgs/msg';
import { describe, expect, it } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { LiftsTable } from './lifts-table';
import { makeLift, makeLiftState } from './test-utils.test';

describe('LiftsTable', () => {
  const rmfApi = new MockRmfApi();
  // mock out some api calls so they never resolves
  rmfApi.fleetsApi.getFleetsFleetsGet = () => new Promise(() => {});
  rmfApi.liftsApi.postLiftRequestLiftsLiftNameRequestPost = () => new Promise(() => {});
  const Base = (props: React.PropsWithChildren<{}>) => {
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

  it('renders with mock lift in AGV', async () => {
    const root = render(
      <Base>
        <LiftsTable />
      </Base>,
    );

    const mockBuildingMap: BuildingMap = {
      name: 'test_map',
      levels: [],
      lifts: [
        makeLift({
          name: 'test_lift2',
          levels: ['L1', 'L2', 'L3'],
        }),
      ],
    };

    act(() => {
      rmfApi.buildingMapObs.next(mockBuildingMap);
    });

    // Create the subject for the lift
    rmfApi.getLiftStateObs('test_lift2');
    const mockLiftState: LiftState = makeLiftState({
      lift_name: 'test_lift2',
      current_floor: 'L2',
      destination_floor: 'L1',
    });
    act(() => {
      rmfApi.liftStateObsStore['test_lift2'].next(mockLiftState);
    });

    expect(root.getByText('Name')).toBeTruthy();
    expect(root.getByText('Op. Mode')).toBeTruthy();
    expect(root.getByText('Current Floor')).toBeTruthy();
    expect(root.getByText('Destination Floor')).toBeTruthy();
    expect(root.getByText('Lift State')).toBeTruthy();

    expect(root.getByText('test_lift2')).toBeTruthy();
    expect(root.getByText('AGV')).toBeTruthy();
    expect(root.getByText('L2')).toBeTruthy();
    expect(root.getByText('L1')).toBeTruthy();
  });

  it('renders with mock lift in HUMAN', async () => {
    const root = render(
      <Base>
        <LiftsTable />
      </Base>,
    );

    const mockBuildingMap: BuildingMap = {
      name: 'test_map',
      levels: [],
      lifts: [
        makeLift({
          name: 'test_lift2',
          levels: ['L1', 'L2', 'L3'],
        }),
      ],
    };

    act(() => {
      rmfApi.buildingMapObs.next(mockBuildingMap);
    });

    // Create the subject for the lift
    rmfApi.getLiftStateObs('test_lift2');
    const mockLiftState2: LiftState = makeLiftState({
      lift_name: 'test_lift2',
      current_floor: 'L1',
      destination_floor: 'L3',
      current_mode: RmfLiftState.MODE_HUMAN,
    });
    act(() => {
      rmfApi.liftStateObsStore['test_lift2'].next(mockLiftState2);
    });

    expect(root.getByText('Name')).toBeTruthy();
    expect(root.getByText('Op. Mode')).toBeTruthy();
    expect(root.getByText('Current Floor')).toBeTruthy();
    expect(root.getByText('Destination Floor')).toBeTruthy();
    expect(root.getByText('Lift State')).toBeTruthy();

    expect(root.getByText('test_lift2')).toBeTruthy();
    expect(root.getByText('HUMAN')).toBeTruthy();
    expect(root.getByText('L1')).toBeTruthy();
    expect(root.getByText('L3')).toBeTruthy();
  });
});
