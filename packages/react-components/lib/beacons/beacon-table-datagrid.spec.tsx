import React from 'react';
import { render } from '@testing-library/react';
import { BeaconDataGridTable } from './beacon-table-datagrid';
import { makeBeaconState } from './test-utils.spec';

describe('BeaconDataGridTable', () => {
  it('renders basic beacons data correctly', () => {
    const root = render(<BeaconDataGridTable beacons={[makeBeaconState()]} />);
    const beaconName = root.getByText('test_beacon');
    const opMode = root.getByText('ONLINE');
    const levelName = root.getByText('L3');
    const beaconType = root.getByText('Audio, Visual');
    const activatedState = root.getByText('OFF');

    expect(beaconName).toBeTruthy();
    expect(opMode).toBeTruthy();
    expect(levelName).toBeTruthy();
    expect(beaconType).toBeTruthy();
    expect(activatedState).toBeTruthy();
  });

  it('renders offline beacon correctly', () => {
    const root = render(
      <BeaconDataGridTable
        beacons={[
          makeBeaconState({
            online: false,
            activated: false,
          }),
        ]}
      />,
    );
    const beaconName = root.getByText('test_beacon');
    const opMode = root.getByText('OFFLINE');
    const levelName = root.getByText('L3');
    const beaconType = root.getByText('Audio, Visual');
    const activatedState = root.getByText('OFF');

    expect(beaconName).toBeTruthy();
    expect(opMode).toBeTruthy();
    expect(levelName).toBeTruthy();
    expect(beaconType).toBeTruthy();
    expect(activatedState).toBeTruthy();
  });

  it('renders activated beacon correctly', () => {
    const root = render(
      <BeaconDataGridTable
        beacons={[
          makeBeaconState({
            activated: true,
          }),
        ]}
      />,
    );
    const beaconName = root.getByText('test_beacon');
    const opMode = root.getByText('ONLINE');
    const levelName = root.getByText('L3');
    const beaconType = root.getByText('Audio, Visual');
    const activatedState = root.getByText('ON');

    expect(beaconName).toBeTruthy();
    expect(opMode).toBeTruthy();
    expect(levelName).toBeTruthy();
    expect(beaconType).toBeTruthy();
    expect(activatedState).toBeTruthy();
  });

  it('shows the correct number of rows', () => {
    const beacons = [makeBeaconState({ id: 'test1' }), makeBeaconState({ id: 'test2' })];
    const root = render(<BeaconDataGridTable beacons={beacons} />);
    const allRows = root.container.querySelectorAll('.MuiDataGrid-row').length;
    expect(allRows).toBe(2);
  });

  it('shows titles correctly', () => {
    const root = render(<BeaconDataGridTable beacons={[makeBeaconState()]} />);
    expect(root.queryByText('Name')).toBeTruthy();
    expect(root.queryByText('Op. Mode')).toBeTruthy();
    expect(root.queryByText('Level')).toBeTruthy();
    expect(root.queryByText('Type')).toBeTruthy();
    expect(root.queryByText('Beacon State')).toBeTruthy();
  });
});
