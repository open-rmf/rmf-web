import { ExpansionPanelDetails, ExpansionPanelSummary } from '@material-ui/core';
import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import fakeDispenserStates from '../mock/data/dispenser-states';
import DispenserItem from './dispenser-item';
import DispenserPanel from './dispensers-panel';

const mount = createMount();

let dispenserStates: Record<string, RomiCore.DispenserState>;

beforeEach(() => {
  dispenserStates = fakeDispenserStates();
});

it('renders dispensers', () => {
  const root = mount(<DispenserPanel dispenserStates={dispenserStates} />);
  const dispenserElements = root.find(DispenserItem);
  expect(dispenserElements.length).toBe(Object.keys(dispenserStates).length);
  root.unmount();
});
