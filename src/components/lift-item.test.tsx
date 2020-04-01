import { MenuItem } from '@material-ui/core';
import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Enzyme from 'enzyme';
import Adapter from 'enzyme-adapter-react-16';
import React from 'react';
import buildingMap from '../mock/data/building-map';
import fakeLiftStates from '../mock/data/lift-states';
import { LiftItem } from './lift-item';

Enzyme.configure({ adapter: new Adapter() });
const mount = createMount();

let map: RomiCore.BuildingMap;
let liftState: RomiCore.LiftState;
let lift: RomiCore.Lift;

beforeEach(async () => {
  map = await buildingMap();
  lift = map.lifts[0];
  liftState = fakeLiftStates()[lift.name];
});

it('responds to request click', () => {
  let requested = false;
  const root = mount(
    <LiftItem
      lift={lift}
      liftState={liftState}
      enableRequest={true}
      onRequest={() => (requested = true)}
    />,
  );

  const requestButton = root.findWhere(x => x.name() === 'button' && x.text() === 'Request');
  requestButton.simulate('click', { clientX: 10, clientY: 10 });
  const requestItem = root.find(MenuItem).filterWhere(x => x.text() === lift.levels[0]);
  requestItem.simulate('click');

  expect(requested).toBe(true);
  root.unmount();
});
