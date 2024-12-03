import { BuildingMap, DoorState } from 'api-client';
import React, { act } from 'react';
import { Door as RmfDoor } from 'rmf-models/ros/rmf_building_map_msgs/msg';
import { DoorMode as RmfDoorMode } from 'rmf-models/ros/rmf_door_msgs/msg';
import { describe, expect, it } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { DoorsTable } from './doors-table';

describe('DoorsTable', () => {
  const rmfApi = new MockRmfApi();
  // mock out some api calls so they never resolves
  rmfApi.doorsApi.postDoorRequestDoorsDoorNameRequestPost = () => new Promise(() => {});
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders with empty doors table', () => {
    const root = render(
      <Base>
        <DoorsTable />
      </Base>,
    );
    expect(root.getByText('Name')).toBeTruthy();
    expect(root.getByText('Op. Mode')).toBeTruthy();
    expect(root.getByText('Current Floor')).toBeTruthy();
    expect(root.getByText('Type')).toBeTruthy();
    expect(root.getByText('Door State')).toBeTruthy();
  });

  it('renders with mock door', async () => {
    const root = render(
      <Base>
        <DoorsTable />
      </Base>,
    );

    const mockBuildingMap: BuildingMap = {
      name: 'test_map',
      levels: [
        {
          name: 'L2',
          elevation: 10,
          images: [],
          places: [],
          doors: [
            {
              name: 'test_door2',
              v1_x: 0,
              v1_y: 0,
              v2_x: 0,
              v2_y: 0,
              door_type: RmfDoor.DOOR_TYPE_DOUBLE_SWING,
              motion_range: 0,
              motion_direction: 0,
            },
          ],
          nav_graphs: [],
          wall_graph: {
            name: 'test_graph',
            vertices: [],
            edges: [],
            params: [],
          },
        },
      ],
      lifts: [],
    };

    act(() => {
      rmfApi.buildingMapObs.next(mockBuildingMap);
    });

    // Create the subject for the door
    rmfApi.getDoorStateObs('test_door2');
    const mockDoorState: DoorState = {
      door_time: { sec: 0, nanosec: 0 },
      door_name: 'test_door2',
      current_mode: { value: RmfDoorMode.MODE_CLOSED },
    };
    act(() => {
      rmfApi.doorStateObsStore['test_door2'].next(mockDoorState);
    });

    expect(root.getByText('Name')).toBeTruthy();
    expect(root.getByText('Op. Mode')).toBeTruthy();
    expect(root.getByText('Current Floor')).toBeTruthy();
    expect(root.getByText('Type')).toBeTruthy();
    expect(root.getByText('Door State')).toBeTruthy();

    expect(root.getByText('test_door2')).toBeTruthy();
    expect(root.getByText('ONLINE')).toBeTruthy();
    expect(root.getByText('L2')).toBeTruthy();
    expect(root.getByText('Double Swing')).toBeTruthy();
    expect(root.getByText('CLOSED')).toBeTruthy();
  });
});
