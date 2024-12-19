import { fireEvent } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { RobotMutexGroupsTable } from './robot-mutex-group-table';

describe('Robot mutex groups table', () => {
  it('Renders robot mutex groups empty table', () => {
    const rmfApi = new MockRmfApi();
    // mock out some api calls so they never resolves
    rmfApi.fleetsApi.getFleetsFleetsGet = () => new Promise(() => {});
    rmfApi.fleetsApi.unlockMutexGroupFleetsNameUnlockMutexGroupPost = () => new Promise(() => {});
    const Base = (props: React.PropsWithChildren<{}>) => {
      return (
        <TestProviders>
          <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
        </TestProviders>
      );
    };
    const root = render(
      <Base>
        <RobotMutexGroupsTable />
      </Base>,
    );

    expect(root.getByText('Group')).toBeTruthy();
    expect(root.getByText('Locked')).toBeTruthy();
    expect(root.getByText('Waiting')).toBeTruthy();
  });

  it('Renders robot mutex groups table with a locked mutex', async () => {
    const rmfApi = new MockRmfApi();
    // mock out some api calls so they never resolves
    const mockFleets = [
      {
        name: 'fleet_1',
        robots: {
          robot_1: {
            name: 'robot_1',
            mutex_groups: {
              locked: ['mutex_group_1'],
              requesting: [],
            },
          },
        },
      },
      {
        name: 'fleet_2',
        robots: {
          robot_2: {
            name: 'robot_2',
            mutex_groups: {
              locked: [],
              requesting: [],
            },
          },
        },
      },
    ];
    rmfApi.fleetsApi.getFleetsFleetsGet = vi.fn().mockResolvedValue({ data: mockFleets });
    rmfApi.fleetsApi.unlockMutexGroupFleetsNameUnlockMutexGroupPost = () => new Promise(() => {});
    const Base = (props: React.PropsWithChildren<{}>) => {
      return (
        <TestProviders>
          <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
        </TestProviders>
      );
    };
    const root = render(
      <Base>
        <RobotMutexGroupsTable />
      </Base>,
    );

    expect(root.getByText('Group')).toBeTruthy();
    expect(root.getByText('Locked')).toBeTruthy();
    expect(root.getByText('Waiting')).toBeTruthy();

    await root.findByText('mutex_group_1');
    expect(root.getByText('mutex_group_1')).toBeTruthy();
    expect(root.getByText('fleet_1/robot_1')).toBeTruthy();
    expect(root.queryByText('fleet_2/robot_2')).not.toBeTruthy();
  });

  it('Renders robot mutex groups table with a locked mutex that is waited on', async () => {
    const rmfApi = new MockRmfApi();
    // mock out some api calls so they never resolves
    const mockFleets = [
      {
        name: 'fleet_1',
        robots: {
          robot_1: {
            name: 'robot_1',
            mutex_groups: {
              locked: ['mutex_group_1'],
              requesting: [],
            },
          },
        },
      },
      {
        name: 'fleet_2',
        robots: {
          robot_2: {
            name: 'robot_2',
            mutex_groups: {
              locked: [],
              requesting: ['mutex_group_1'],
            },
          },
        },
      },
    ];
    rmfApi.fleetsApi.getFleetsFleetsGet = vi.fn().mockResolvedValue({ data: mockFleets });
    const mockUnlock = vi.fn().mockImplementation(async () => {
      // Simulate some asynchronous operation that has no return value
      await new Promise((resolve) => setTimeout(resolve, 100));
    });
    // rmfApi.fleetsApi.unlockMutexGroupFleetsNameUnlockMutexGroupPost = () => new Promise(() => {});
    rmfApi.fleetsApi.unlockMutexGroupFleetsNameUnlockMutexGroupPost = mockUnlock;
    const Base = (props: React.PropsWithChildren<{}>) => {
      return (
        <TestProviders>
          <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
        </TestProviders>
      );
    };
    const root = render(
      <Base>
        <RobotMutexGroupsTable />
      </Base>,
    );

    expect(root.getByText('Group')).toBeTruthy();
    expect(root.getByText('Locked')).toBeTruthy();
    expect(root.getByText('Waiting')).toBeTruthy();

    await root.findByText('mutex_group_1');
    expect(root.getByText('mutex_group_1')).toBeTruthy();
    expect(root.getByText('fleet_1/robot_1')).toBeTruthy();
    expect(root.getByText('fleet_2/robot_2')).toBeTruthy();

    // Try to unlock
    fireEvent.click(root.getByText('mutex_group_1'));
    expect(
      root.getByText('Confirm unlock mutex group [mutex_group_1] for [fleet_1/robot_1]?'),
    ).toBeTruthy();
    expect(root.getByText('Confirm unlock')).toBeTruthy();
    fireEvent.click(root.getByText('Confirm unlock'));
    expect(mockUnlock).toBeCalled();
  });
});
