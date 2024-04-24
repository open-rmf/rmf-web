import React from 'react';
import { BeaconDataGridTable } from './beacon-table-datagrid';
import { makeBeaconState } from './test-utils.spec';

export default {
  title: 'BeaconDataGridTable',
};

export const Default = () => (
  <BeaconDataGridTable
    beacons={[
      makeBeaconState({ id: 'test_beacon1' }),
      makeBeaconState({ id: 'test_beacon2', activated: true }),
      makeBeaconState({ id: 'test_beacon3', online: false }),
    ]}
  />
);

export const Empty = () => <BeaconDataGridTable beacons={[]} />;
