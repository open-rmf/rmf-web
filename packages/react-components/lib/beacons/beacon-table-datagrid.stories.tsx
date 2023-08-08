import React from 'react';
import { BeaconDataGridTable } from './beacon-table-datagrid';
import { storiesOf } from '@storybook/react';
import { makeBeaconState } from './test-utils.spec';

storiesOf('BeaconDataGridTable', module)
  .add('Default', () => <BeaconDataGridTable beacons={[makeBeaconState()]} />)
  .add('Empty', () => <BeaconDataGridTable beacons={[]} />);
