import React from 'react';
import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import DispensersPanel from './dispensers-panel';
import fakeDispenserStates from '../mock/data/dispenser-states';
import FakeTransport from '../mock/fake-transport';

const mount = createMount();

let dispenserStates: Record<string, RomiCore.DispenserState>;
let transport: FakeTransport;

beforeEach(async() => {
  dispenserStates = fakeDispenserStates();
  transport = new FakeTransport();
});

it('fires on dispenser click event', () => {
  let clicked = false;
  const root = mount(
    <DispensersPanel dispenserStates={dispenserStates} 
        onDispenserClick={() => (clicked = true)} />
});



