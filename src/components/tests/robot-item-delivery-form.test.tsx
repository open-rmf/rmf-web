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
        .find(`#robot1-pickup-dispenser`)
        .find('input')
        .props().value,
    ).toBe('');

    expect(
      wrapper
        .find(`#robot1-dropoff-dispenser`)
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
        .find(`#robot1-pickup-dispenser`)
        .find('#robot1-pickup-dispenser-helper-text')
        .first()
        .find('p')
        .props().children,
    ).toBe('Cannot be empty');

    expect(
      wrapper
        .find(`#robot1-dropoff-dispenser`)
        .find('#robot1-dropoff-dispenser-helper-text')
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
        .find('#robot1-dropoff-place')
        .find('#robot1-dropoff-place-helper-text')
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

  test('Start Location cannot be equal to finish Location', async () => {});
  test('Initial values with places with dispensers ', () => {});
  test('Error shows with places without dispensers ', () => {});
});
