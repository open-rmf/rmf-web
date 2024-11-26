import { waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { DoorState } from 'api-client';
import React, { act } from 'react';
import { Door as RmfDoor } from 'rmf-models/ros/rmf_building_map_msgs/msg';
import { DoorMode as RmfDoorMode } from 'rmf-models/ros/rmf_door_msgs/msg';
import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { DoorSummary } from './door-summary';
import { makeDoor } from './test-utils.test';

describe('DoorSummary', () => {
  const mockDoor: RmfDoor = makeDoor({ name: 'test_door' });

  const rmfApi = new MockRmfApi();
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders door summary correctly', async () => {
    const onCloseMock = vi.fn();
    const root = render(
      <Base>
        <DoorSummary onClose={onCloseMock} door={mockDoor} doorLevelName="L1" />
      </Base>,
    );

    // Create the subject for the door
    const doorStateObs = rmfApi.getDoorStateObs('test_door');
    let emittedDoorState: DoorState | undefined;
    doorStateObs.subscribe((doorState) => {
      emittedDoorState = doorState;
    });

    const mockDoorState: DoorState = {
      door_time: { sec: 0, nanosec: 0 },
      door_name: 'test_door',
      current_mode: { value: RmfDoorMode.MODE_OPEN },
    };
    act(() => {
      rmfApi.doorStateObsStore['test_door'].next(mockDoorState);
    });

    expect(emittedDoorState).toEqual(mockDoorState);
    expect(emittedDoorState?.current_mode.value).toEqual(RmfDoorMode.MODE_OPEN);

    expect(root.getByText('Name')).toBeTruthy();
    expect(root.getByText('Current Floor')).toBeTruthy();
    expect(root.getByText('Type')).toBeTruthy();
    expect(root.getByText('State')).toBeTruthy();

    expect(root.getByText('test_door')).toBeTruthy();
    expect(root.getByText('L1')).toBeTruthy();
    expect(root.getByText('Single Swing')).toBeTruthy();
    expect(root.getByText('OPEN')).toBeTruthy();

    userEvent.keyboard('{Escape}');
    await waitFor(() => expect(onCloseMock).toHaveBeenCalledTimes(1));
  });
});
