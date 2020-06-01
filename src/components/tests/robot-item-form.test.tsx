import { Button } from '@material-ui/core';
import { createMount } from '@material-ui/core/test-utils';
import { RobotLoopForm } from '../robot-item-form';
import fakePlaces from '../../mock/data/places';
import React from 'react';

const mount = createMount();

const buildWrapper = () => {
  const onClick = (
    fleetName: string,
    numLoops: number,
    startLocationPoint: string,
    endLocationPoint: string,
  ) => {
    console.log('test');
  };

  const wrapper = mount(
    <RobotLoopForm
      requestLoop={onClick}
      fleetName={'SuperFleet'}
      listOfPlaces={fakePlaces()['SuperFleet']}
    />,
  );
  return wrapper;
};

describe('form Validation', () => {
  test('Initial Values', () => {
    const wrapper = buildWrapper();
    expect(wrapper.find("input[type='number']").props().value).toEqual('');
    expect(wrapper.findWhere(x => x.name() === 'input' && x.props().value === 'supplies'))
      .toBeTruthy;
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
    wrapper.unmount();
  });

  test('If the for isn`t completed show an error', async () => {
    const wrapper = buildWrapper();
    wrapper.find('form').simulate('submit');
    expect(wrapper.find('#numLoopsError').html()).toContain('Loops can only be &gt; 0');
    wrapper.unmount();
  });
});
