import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import LiftRequestForm from '../lift-item-form';
import { LiftRequestManager } from '../../../lift-state-manager';

const mount = createMount();

const buildWrapper = () => {
  const onClick = (doorState: number, requestType: number, destination: string) => {
    console.log('test');
  };

  const wrapper = mount(
    <LiftRequestForm
      liftRequest={onClick}
      doorStates={LiftRequestManager.getDoorModes()}
      requestTypes={LiftRequestManager.getLiftRequestModes()}
      destinationList={['L1', 'L2']}
    />,
  );
  return wrapper;
};

describe('form Validation', () => {
  test('Initial Values', () => {
    const wrapper = buildWrapper();
    expect(
      wrapper.findWhere(
        x =>
          x.name() === 'input' &&
          x.props().value === LiftRequestManager.doorStateToString(RomiCore.LiftState.DOOR_OPEN),
      ),
    ).toBeTruthy;

    expect(
      wrapper.findWhere(
        x =>
          x.name() === 'input' &&
          x.props().value ===
            LiftRequestManager.requestModeToString(RomiCore.LiftRequest.REQUEST_AGV_MODE),
      ),
    ).toBeTruthy;
    wrapper.unmount();
  });

  test('Destination cannot be empty', async () => {
    const wrapper = buildWrapper();
    wrapper.find('form').simulate('submit');
    expect(
      wrapper
        .find('#destination')
        .find('p')
        .html(),
    ).toContain('Destination cannot be empty');
    wrapper.unmount();
  });
});
