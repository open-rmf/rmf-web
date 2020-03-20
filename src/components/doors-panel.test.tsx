import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Enzyme, { ReactWrapper } from 'enzyme';
import Adapter from 'enzyme-adapter-react-16';
import React from 'react';
import buildingMap from '../mock/data/building-map';
import fakeDoorStates from '../mock/data/door-states';
import DoorsPanel from './doors-panel';

Enzyme.configure({ adapter: new Adapter() });
const mount = createMount();

let root: ReactWrapper;
let map: RomiCore.BuildingMap;
let doorStates: Record<string, RomiCore.DoorState>;
let doors: RomiCore.Door[];

beforeAll(async () => {
  map = await buildingMap();
  doorStates = fakeDoorStates();
  doors = map.levels.flatMap(l => l.doors);
});

beforeEach(() => {
  root = mount(<DoorsPanel doorStates={doorStates} doors={doors} />);
});

afterEach(() => {
  root.unmount();
});

it('renders doors', () => {
  const doorNames = doors.reduce<Record<string, boolean>>(
    (prev, door) => (prev[door.name] = true) && prev,
    {},
  );
  const doorElements = root.findWhere(x => x.name() === null && doorNames[x.text()]);
  expect(doorElements.length).toBe(doors.length);
});

it('expands on click', () => {
  const doorElement = root.findWhere(x => x.text() === doors[0].name).at(0);

  // expansion details should be unmount at the start
  expect(root.findWhere(x => x.text().startsWith('Motion Direction')).length).toBeFalsy();

  doorElement.simulate('click');

  // now the details should be mounted and visible
  expect(root.findWhere(x => x.text().startsWith('Motion Direction')).length).toBeTruthy();
});
