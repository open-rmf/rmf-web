import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import fakeFleets from '../../mock/data/fleets';
import CommandsPanel from '../commands-panel';
import { LoopForm } from '../loop-form';
import { ResourcesContext } from '../../app-contexts';
import ResourceManager from '../../resource-manager';
import fakeResources from '../../mock/data/resources';
import { RobotDeliveryForm } from '../delivery-form';

const mount = createMount();

let fleets: RomiCore.FleetState[];

beforeEach(() => {
  fleets = fakeFleets();
});

it('Renders loop and delivery form', () => {
  const resources = new ResourceManager(fakeResources());
  const root = mount(
    <ResourcesContext.Provider value={resources}>
      <CommandsPanel allFleets={fleets.map(fleet => fleet.name)} />
    </ResourcesContext.Provider>,
  );

  expect(root.find(LoopForm).length).toBe(1);
  expect(root.find(RobotDeliveryForm).length).toBe(1);

  root.unmount();
});
