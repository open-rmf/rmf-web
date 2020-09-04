import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import fakeResources from '../../../mock/data/resources';
import ResourceManager, { ResourceConfigurationsType } from '../../../resource-manager';
import Dispenser from '../dispenser';
import fakeDispenserStates from '../../../mock/data/dispenser-states';
import { ResourcesContext } from '../../app-contexts';
import toJson from 'enzyme-to-json';
import { DispenserResourceManager } from '../../../resource-manager-dispensers';

const mount = createMount();

describe('Dispenser component', () => {
  let dispenserHandler: DispenserResourceManager;

  beforeEach(() => {
    const dispenserResources = fakeResources().dispensers;
    if (!dispenserResources || !Object.keys(dispenserResources).length) {
      throw new Error('To execute this test you need to instantiate a dispenser');
    }
    dispenserHandler = new DispenserResourceManager(dispenserResources);
  });

  test('Fires click event when it has a state', () => {
    let clicked = false;

    const dispenserStates = fakeDispenserStates();
    const dispensers = dispenserHandler.allValues;
    const root = mount(
      <svg>
        <Dispenser
          dispenser={dispensers[0]}
          dispenserHandler={dispenserHandler}
          footprint={0.5}
          onClick={() => (clicked = true)}
          dispenserState={dispenserStates[dispensers[0].guid]}
        />
      </svg>,
    );

    root.find(Dispenser).simulate('click');
    expect(clicked).toBe(true);

    root.unmount();
  });

  test(`Don't Fire click event when it has no state`, () => {
    let clicked = false;
    const dispensers = dispenserHandler.allValues;
    const root = mount(
      <svg>
        <Dispenser
          dispenser={dispensers[0]}
          footprint={0.5}
          onClick={() => (clicked = true)}
          dispenserHandler={dispenserHandler}
        />
      </svg>,
    );

    root.find(Dispenser).simulate('click');
    expect(clicked).toBe(false);

    root.unmount();
  });

  test(`Renders correctly`, () => {
    const dispensers = dispenserHandler.allValues;
    const wrapper = mount(
      <svg>
        <Dispenser dispenser={dispensers[0]} footprint={0.5} dispenserHandler={dispenserHandler} />
      </svg>,
    );

    expect(toJson(wrapper)).toMatchSnapshot();
  });
});
