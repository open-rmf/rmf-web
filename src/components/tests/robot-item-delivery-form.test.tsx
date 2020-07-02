import { createMount } from '@material-ui/core/test-utils';
import { RobotDeliveryForm } from '../robot-item-delivery-form';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import fakePlaces from '../../mock/data/places';
import React from 'react';

const mount = createMount();

const buildWrapper = (listOfPlaces?: string[]) => {
  const onClick = (
    pickupPlaceName: string,
    pickupDispenser: string,
    dropOffPlaceName: string,
    dropOffDispenser: string,
    pickupBehaviour?: RomiCore.Behavior,
    dropOffBehavior?: RomiCore.Behavior,
  ) => {
    console.log('test');
  };

  const wrapper = mount(
    <RobotDeliveryForm
      requestDelivery={onClick}
      robotName={'robot1'}
      fleetName={'SuperFleet'}
      listOfPlaces={!!listOfPlaces ? listOfPlaces : fakePlaces()['SuperFleet']}
    />,
  );
  return wrapper;
};

describe('Form validation', () => {
  test('Initial values', () => {
    const wrapper = buildWrapper();
    expect(
      wrapper.findWhere(
        x => x.name() === 'input' && x.props().value === fakePlaces()['SuperFleet'][0],
      ),
    ).toBeTruthy;

    expect(
      wrapper.findWhere(
        x => x.name() === 'input' && x.props().value === fakePlaces()['SuperFleet'][1],
      ),
    ).toBeTruthy;

    expect(
      wrapper
        .find(`#robot1PickupDispenser`)
        .find('input')
        .props().value,
    ).toBe('');

    expect(
      wrapper
        .find(`#robot1DropoffDispenser`)
        .find('input')
        .props().value,
    ).toBe('');

    wrapper.unmount();
  });

  test('Dispensers cannot be empty', () => {
    const wrapper = buildWrapper(['place1', 'place2']);
    wrapper.find('form').simulate('submit');
    expect(
      wrapper
        .find(`#robot1PickupDispenser`)
        .find('#robot1PickupDispenser-helper-text')
        .first()
        .find('p')
        .props().children,
    ).toBe('Cannot be empty');

    expect(
      wrapper
        .find(`#robot1DropoffDispenser`)
        .find('#robot1DropoffDispenser-helper-text')
        .first()
        .find('p')
        .props().children,
    ).toEqual('Cannot be empty');
    wrapper.unmount();
  });

  test('Place cannot be empty', async () => {
    const wrapper = buildWrapper(['placeA']);
    wrapper.find('form').simulate('submit');
    expect(
      wrapper
        .find('#robot1DropoffPlace')
        .find('#robot1DropoffPlace-helper-text')
        .first()
        .html(),
    ).toContain('Cannot be empty');
    wrapper.unmount();
  });

  test('Start place cannot be equal to finish place', async () => {
    const wrapper = buildWrapper(['placeA', 'placeA', 'placeA']);
    wrapper.find('form').simulate('submit');
    wrapper.unmount();
  });

  test('Initial values with places with dispensers ', () => {
    const wrapper = buildWrapper(['hardware_2', 'pantry']);
    expect(wrapper.find('#robot1PickupDispenser-helper-text').exists()).toBe(false);
    expect(wrapper.find('#robot1DropoffDispenser-helper-text').exists()).toBe(false);
    wrapper.unmount();
  });

  test('Error shows with places without dispensers ', () => {
    const wrapper = buildWrapper(['place1', 'place2']);

    expect(
      wrapper
        .find('#robot1PickupDispenser-helper-text')
        .first()
        .html(),
    ).toContain('There is no dispensers on this place. Pick another place');

    expect(
      wrapper
        .find('#robot1DropoffDispenser-helper-text')
        .first()
        .html(),
    ).toContain('There is no dispensers on this place. Pick another place');

    wrapper.unmount();
  });
});

test('Start Location cannot be equal to finish Location', async () => {
  const wrapper = buildWrapper();
});
