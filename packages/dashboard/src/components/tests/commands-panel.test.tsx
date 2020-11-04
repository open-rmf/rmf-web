import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import fakeFleets from '../../mock/data/fleets';
import CommandsPanel from '../commands-panel';

const mount = createMount();

let fleets: RomiCore.FleetState[];

beforeEach(() => {
  fleets = fakeFleets();
});

it('Renders error on render without context', () => {
  const root = mount(<CommandsPanel allFleets={fleets.map((fleet) => fleet.name)} />);
  expect(root.find('#no-config-file-error-msg').exists()).toBeTruthy();
  root.unmount();
});
