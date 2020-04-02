import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import buildingMap from '../mock/data/building-map';
import { PlaceItem } from './place-item';
import PlacesPanel from './places-panel';

const mount = createMount();

let map: RomiCore.BuildingMap;
let places: RomiCore.Place[];

beforeEach(async () => {
  map = await buildingMap();
  places = map.levels.flatMap(l => l.places);
});

it('renders places', () => {
  const root = mount(<PlacesPanel buildingMap={map} />);
  const placeElements = root.find(PlaceItem);
  expect(placeElements.length).toBe(places.length);
  root.unmount();
});

it('fires click event', () => {
  let clicked = false;
  const root = mount(<PlacesPanel buildingMap={map} onPlaceClick={() => (clicked = true)} />);
  const placeElement = root.find(PlaceItem).at(0);
  placeElement.simulate('click');
  expect(clicked).toBe(true);
  root.unmount();
});
