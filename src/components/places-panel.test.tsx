import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Enzyme, { ReactWrapper } from 'enzyme';
import Adapter from 'enzyme-adapter-react-16';
import React from 'react';
import buildingMap from '../mock/data/building-map';
import PlacesPanel from './places-panel';

Enzyme.configure({ adapter: new Adapter() });
const mount = createMount();

let root: ReactWrapper;
let map: RomiCore.BuildingMap;
let places: RomiCore.Place[];

beforeAll(async () => {
  map = await buildingMap();
  places = map.levels.flatMap(l => l.places);
});

beforeEach(() => {
  root = mount(<PlacesPanel buildingMap={map} />);
});

afterEach(() => {
  root.unmount();
});

it('renders doors', () => {
  const placeNames = map.levels
    .flatMap(l => l.places)
    .reduce<Record<string, boolean>>((prev, door) => (prev[door.name] = true) && prev, {});
  const placeElements = root.findWhere(x => x.name() === null && placeNames[x.text()]);
  expect(placeElements.length).toBe(places.length);
});

it('expands on click', () => {
  const placeElement = root.findWhere(x => x.text() === places[0].name).at(0);

  // expansion details should be unmount at the start
  expect(root.findWhere(x => x.text().startsWith('Position Tolerance')).length).toBeFalsy();

  placeElement.simulate('click');

  // now the details should be mounted and visible
  expect(root.findWhere(x => x.text().startsWith('Position Tolerance')).length).toBeTruthy();
});
