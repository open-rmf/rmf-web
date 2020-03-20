import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Enzyme, { ReactWrapper } from 'enzyme';
import Adapter from 'enzyme-adapter-react-16';
import React from 'react';
import fakeFleets from '../mock/data/fleets';
import RobotsPanel from './robots-panel';

Enzyme.configure({ adapter: new Adapter() });
const mount = createMount();

let root: ReactWrapper;
let fleets: RomiCore.FleetState[];
let robots: RomiCore.RobotState[];

beforeEach(() => {
  fleets = fakeFleets();
  robots = fleets.flatMap(x => x.robots);
  root = mount(<RobotsPanel fleets={fleets} />);
});

afterEach(() => {
  root.unmount();
});

it('renders robots', () => {
  const robotNames = robots.reduce<Record<string, boolean>>(
    (prev, robot) => (prev[robot.name] = true) && prev,
    {},
  );
  const robotElements = root.findWhere(x => x.name() === null && robotNames[x.text()]);
  expect(robotElements.length).toBe(robots.length);
});

it('expands on click', () => {
  const robotElement = root.findWhere(x => x.text() === robots[0].name).at(0);

  // expansion details should be unmount at the start
  expect(root.findWhere(x => x.text().startsWith('Model')).length).toBeFalsy();

  robotElement.simulate('click');

  // now the details should be mounted and visible
  expect(root.findWhere(x => x.text().startsWith('Model')).length).toBeTruthy();
});
