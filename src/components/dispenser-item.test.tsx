import {
  ListItem,
} from '@material-ui/core';
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

it('lists dispensers queued requests ID', () => {
  Object.keys(dispenserStates).map( (guid, index) => {
    const state = dispenserStates[guid];

    const root = mount(<DispenserItem dispenserState={state} />);
    const QueuedIds = root.find(ListItem);
    expect(QueuedIds.length).toBe(state.request_guid_queue.length);
    root.unmount();
  });
});
