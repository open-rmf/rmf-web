import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Enzyme, { ReactWrapper } from 'enzyme';
import Adapter from 'enzyme-adapter-react-16';
import React from 'react';
import buildingMap from '../mock/data/building-map';
import fakeLiftStates from '../mock/data/lift-states';
import LiftsPanel from './lifts-panel';

Enzyme.configure({ adapter: new Adapter() });
const mount = createMount();

let root: ReactWrapper;
let map: RomiCore.BuildingMap;
let liftStates: Record<string, RomiCore.LiftState>;
let lifts: RomiCore.Lift[];

beforeAll(async () => {
  map = await buildingMap();
  liftStates = fakeLiftStates();
  lifts = map.lifts;
});

beforeEach(() => {
  root = mount(<LiftsPanel liftStates={liftStates} lifts={lifts} />);
});

afterEach(() => {
  root.unmount();
});

it('renders lifts', () => {
  const liftNames = lifts.reduce<Record<string, boolean>>(
    (prev, door) => (prev[door.name] = true) && prev,
    {},
  );
  const doorElements = root.findWhere(x => x.name() === null && liftNames[x.text()]);
  expect(doorElements.length).toBe(lifts.length);
});

it('expands on click', () => {
  const liftElement = root.findWhere(x => x.text() === lifts[0].name).at(0);

  // expansion details should be unmount at the start
  expect(root.findWhere(x => x.text().startsWith('Destination Floor')).length).toBeFalsy();

  liftElement.simulate('click');

  // now the details should be mounted and visible
  expect(root.findWhere(x => x.text().startsWith('Destination Floor')).length).toBeTruthy();
});
