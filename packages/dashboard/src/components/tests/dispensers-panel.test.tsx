import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import fakeDispenserStates from '../../mock/data/dispenser-states';
import DispenserItem from '../dispenser-item';
import DispenserPanel from '../dispensers-panel';
import { ResourcesContext } from '../app-contexts';
import fakeResources from '../../mock/data/resources';
import ResourceManager from '../../resource-manager';

const mount = createMount();

let dispenserStates: Record<string, RomiCore.DispenserState>;

beforeEach(() => {
  dispenserStates = fakeDispenserStates();
});

it('renders dispensers', () => {
  const resources = new ResourceManager(fakeResources());
  const root = mount(
    <ResourcesContext.Provider value={resources}>
      <DispenserPanel dispenserStates={dispenserStates} />
    </ResourcesContext.Provider>,
  );
  const dispenserElements = root.find(DispenserItem);
  const dispenersInResource: number = resources.dispensers
    ? Object.keys(resources.dispensers.dispensers).length
    : 0;
  expect(dispenserElements.length).toBe(dispenersInResource);
  root.unmount();
});
