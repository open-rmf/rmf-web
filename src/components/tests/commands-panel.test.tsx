import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import fakeFleets from '../../mock/data/fleets';
import CommandsPanel from '../commands-panel';
import { LoopForm } from '../loop-form';

const mount = createMount();

let fleets: RomiCore.FleetState[];

beforeEach(() => {
  fleets = fakeFleets();
});

it('renders loop form', () => {
  const root = mount(<CommandsPanel fleets={fleets} />);
  const formElements = root.find(LoopForm);
  expect(formElements.length).toBe(1);
  root.unmount();
});
