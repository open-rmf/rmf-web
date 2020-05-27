import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import buildingMap from '../../mock/data/building-map';
import fakeDoorStates from '../../mock/data/door-states';
import DoorItem from '../door-item';

const mount = createMount();

let map: RomiCore.BuildingMap;
let doorState: RomiCore.DoorState;
let door: RomiCore.Door;

beforeEach(async () => {
  map = await buildingMap();
  door = map.levels.flatMap(l => l.doors)[0];
  doorState = fakeDoorStates()[door.door_name];
});

it('responds to open click', () => {
  let clicked = false;
  const root = mount(
    <DoorItem
      door={door}
      doorState={doorState}
      enableControls={true}
      onOpenClick={() => (clicked = true)}
    />,
  );

  const openButton = root.findWhere(x => x.name() === 'button' && x.text() === 'Open');
  openButton.simulate('click');

  expect(clicked).toBe(true);
  root.unmount();
});

it('responds to close click', () => {
  let clicked = false;
  const root = mount(
    <DoorItem
      door={door}
      doorState={doorState}
      enableControls={true}
      onCloseClick={() => (clicked = true)}
    />,
  );

  const closeButton = root.findWhere(x => x.name() === 'button' && x.text() === 'Close');
  closeButton.simulate('click');

  expect(clicked).toBe(true);
  root.unmount();
});
