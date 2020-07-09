import { createMount } from '@material-ui/core/test-utils';
import { LoopForm } from '../loop-form';
import fakePlaces from '../../mock/data/places';
import React from 'react';

const mount = createMount();

const buildWrapper = (fleetName: string) => {
  const onClick = (
    fleetName: string,
    numLoops: number,
    startLocationPoint: string,
    endLocationPoint: string,
  ) => {
    console.log('test');
  };

  const wrapper = mount(<LoopForm requestLoop={onClick} fleets={[fleetName]} />);
  return wrapper;
};

describe('form Validation', () => {
  test('Initial Values', () => {
    const wrapper = buildWrapper('SuperFleet');
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

  test('Number of loops cannot be empty', async () => {
    const wrapper = buildWrapper('SuperFleet');
    wrapper.find('form').simulate('submit');
    expect(wrapper.find('#numLoopsError').html()).toContain('Loops can only be &gt; 0');
    wrapper.unmount();
  });

  test('Location cannot be empty', async () => {
    const wrapper = buildWrapper('FleetA');
    wrapper.find('form').simulate('submit');
    expect(wrapper.find('#startLocationError').html()).toContain('Location cannot be empty');
    wrapper.unmount();
  });

  test('Start Location cannot be equal to finish Location', async () => {
    const wrapper = buildWrapper('FleetB');
    wrapper.find('form').simulate('submit');
    expect(wrapper.find('#startLocationError').html()).toContain(
      'Start Location cannot be equal to Finish Location',
    );
    expect(wrapper.find('#finishLocationError').html()).toContain(
      'Start Location cannot be equal to Finish Location',
    );
    wrapper.unmount();
  });
});
