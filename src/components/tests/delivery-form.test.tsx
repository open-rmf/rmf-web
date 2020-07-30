import { createMount } from '@material-ui/core/test-utils';
import { RobotDeliveryForm } from '../delivery-form';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import fakePlaces from '../../mock/data/places';
import React from 'react';
import { TDeliveryRequest } from '../commands-panel';

const mount = createMount();

const buildWrapper = (fleetName: string, onClick: TDeliveryRequest) => {
  const wrapper = mount(<RobotDeliveryForm requestDelivery={onClick} fleetNames={[fleetName]} />);
  return wrapper;
};

describe('Form validation', () => {
  let isRequestButtonClicked = false;
  const onClick = (
    pickupPlaceName: string,
    pickupDispenser: string,
    dropOffPlaceName: string,
    dropOffDispenser: string,
    pickupBehaviour?: RomiCore.Behavior,
    dropOffBehavior?: RomiCore.Behavior,
  ) => {
    isRequestButtonClicked = true;
  };

  beforeEach(() => {
    isRequestButtonClicked = false;
  });

  test('Initial values', () => {
    const wrapper = buildWrapper('SuperFleet', onClick);
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
        .find(`#pickupDispenser`)
        .find('input')
        .props().value,
    ).toBe('');

    expect(
      wrapper
        .find(`#dropoffDispenser`)
        .find('input')
        .props().value,
    ).toBe('');

    wrapper.unmount();
  });

  test('Dispensers cannot be empty', () => {
    const wrapper = buildWrapper('FleetB', onClick);
    wrapper.find('form').simulate('submit');
    expect(
      wrapper
        .find('#pickupDispenser-helper-text')
        .first()
        .find('p')
        .props().children,
    ).toBe('Cannot be empty');

    expect(
      wrapper
        .find('#dropoffDispenser-helper-text')
        .first()
        .find('p')
        .props().children,
    ).toEqual('Cannot be empty');
    expect(isRequestButtonClicked).toBeFalsy();
    wrapper.unmount();
  });

  test('Place cannot be empty', async () => {
    const wrapper = buildWrapper('FleetA', onClick);
    wrapper.find('form').simulate('submit');
    expect(
      wrapper
        .find('#pickupPlace-helper-text')
        .first()
        .html(),
    ).toContain('Cannot be empty');
    expect(isRequestButtonClicked).toBeFalsy();
    wrapper.unmount();
  });

  test('Initial values with places with dispensers ', () => {
    const wrapper = buildWrapper('TestFleet', onClick);
    expect(wrapper.find('#robot1PickupDispenser-helper-text').exists()).toBe(false);
    expect(wrapper.find('#robot1DropoffDispenser-helper-text').exists()).toBe(false);
    wrapper.unmount();
  });

  test('Error shows with places without dispensers ', () => {
    const wrapper = buildWrapper('Fleet2', onClick);
    expect(
      wrapper
        .find('#pickupDispenser-helper-text')
        .first()
        .html(),
    ).toContain('There is no dispensers on this place. Pick another place');
    expect(
      wrapper
        .find('#dropoffDispenser-helper-text')
        .first()
        .html(),
    ).toContain('There is no dispensers on this place. Pick another place');
    wrapper.unmount();
  });

  test('Start place cannot be equal to finish place', () => {
    const wrapper = buildWrapper('FleetB', onClick);
    wrapper.find('form').simulate('submit');
    expect(
      wrapper
        .find('#pickupPlace-helper-text')
        .first()
        .html(),
    ).toContain('Start Location cannot be equal to finish Location');
    expect(
      wrapper
        .find('#dropoffPlace-helper-text')
        .first()
        .html(),
    ).toContain('Start Location cannot be equal to finish Location');
    expect(isRequestButtonClicked).toBeFalsy();
    wrapper.unmount();
  });
});
