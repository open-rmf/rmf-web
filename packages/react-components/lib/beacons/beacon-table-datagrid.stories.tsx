import React from 'react';
import { BeaconDataGridTable } from './beacon-table-datagrid';
import { storiesOf } from '@storybook/react';
import { makeBeaconState } from './test-utils.spec';

storiesOf('BeaconDataGridTable', module)
  .add('Default', () => (
    <BeaconDataGridTable
      beacons={[
        makeBeaconState({ id: 'test_beacon1' }),
        makeBeaconState({ id: 'test_beacon2', activated: true }),
        makeBeaconState({ id: 'test_beacon3', online: false }),
      ]}
    />
  ))
  .add('Empty', () => <BeaconDataGridTable beacons={[]} />);
