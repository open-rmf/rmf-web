import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import fakeResources from '../../../mock/data/resources';
import ResourceManager, { ResourceConfigurationsType } from '../../../resource-manager';
import Dispenser from '../dispenser';
import fakeDispenserStates from '../../../mock/data/dispenser-states';
import { ResourcesContext } from '../../app-contexts';

const mount = createMount();

describe('Dispenser component', () => {
  const resources = new ResourceManager(fakeResources() as Required<ResourceConfigurationsType>);

  test('Fires click event when it has a state', () => {
    let clicked = false;
    if (!resources.dispensers) {
      throw new Error('To execute this test you need to instantiate a dispenser');
    }
    const dispenserStates = fakeDispenserStates();
    const dispensers = resources.dispensers.allValues;
    const root = mount(
      <ResourcesContext.Provider value={resources}>
        <svg>
          <Dispenser
            dispenser={dispensers[0]}
            footprint={0.5}
            onClick={() => (clicked = true)}
            dispenserState={dispenserStates[dispensers[0].guid]}
          />
        </svg>
      </ResourcesContext.Provider>,
    );

    root.find(Dispenser).simulate('click');
    expect(clicked).toBe(true);

    root.unmount();
  });

  test(`Don't Fire click event when it has no state`, () => {
    let clicked = false;
    if (!resources.dispensers) {
      throw new Error('To execute this test you need to instantiate a dispenser');
    }
    const dispensers = resources.dispensers.allValues;
    const root = mount(
      <ResourcesContext.Provider value={resources}>
        <svg>
          <Dispenser dispenser={dispensers[0]} footprint={0.5} onClick={() => (clicked = true)} />
        </svg>
      </ResourcesContext.Provider>,
    );

    root.find(Dispenser).simulate('click');
    expect(clicked).toBe(false);

    root.unmount();
  });
});
