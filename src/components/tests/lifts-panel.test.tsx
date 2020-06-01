import { MenuItem } from '@material-ui/core';
import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import buildingMap from '../../mock/data/building-map';
import fakeLiftStates from '../../mock/data/lift-states';
import FakeTransport from '../../mock/fake-transport';
import { LiftItem } from '../lift-item';
import LiftsPanel from '../lifts-panel';

const mount = createMount();

let map: RomiCore.BuildingMap;
let liftStates: Record<string, RomiCore.LiftState>;
let lifts: RomiCore.Lift[];
let transport: FakeTransport;

beforeEach(async () => {
  map = await buildingMap();
  liftStates = fakeLiftStates();
  lifts = map.lifts;
  transport = new FakeTransport();
});

it('renders lifts', () => {
  const root = mount(<LiftsPanel liftStates={liftStates} lifts={lifts} />);
  const liftElements = root.find(LiftItem);
  expect(liftElements.length).toBe(lifts.length);
  root.unmount();
});

it('publish request on request click', () => {
  const publishMock = jest.fn();
  jest.spyOn(transport, 'createPublisher').mockImplementation(() => {
    return {
      publish: publishMock,
    };
  });
  const root = mount(<LiftsPanel liftStates={liftStates} lifts={lifts} transport={transport} />);
  const liftElement = root.find(LiftItem).at(0);

  const requestButton = liftElement.findWhere(x => x.name() === 'button' && x.text() === 'Request');
  requestButton.simulate('click', { clientX: 10, clientY: 10 });
  const requestItem = liftElement
    .update()
    .find(MenuItem)
    .at(0);
  requestItem.simulate('click');

  expect(publishMock).toHaveBeenCalledTimes(1);
  root.unmount();
});

it('fires on lift click event', () => {
  let clicked = false;
  const root = mount(
    <LiftsPanel liftStates={liftStates} lifts={lifts} onLiftClick={() => (clicked = true)} />,
  );
  const liftElement = root.find(LiftItem).at(0);
  liftElement.simulate('click');
  expect(clicked).toBe(true);
  root.unmount();
});
