import { ExpansionPanelDetails, ExpansionPanelSummary } from '@material-ui/core';
import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import fakeFleets from '../mock/data/fleets';
import RobotItem from './robot-item';
import RobotsPanel from './robots-panel';

const mount = createMount();

let fleets: RomiCore.FleetState[];
let robots: RomiCore.RobotState[];

beforeEach(() => {
  fleets = fakeFleets();
  robots = fleets.flatMap(x => x.robots);
});

it('renders robots', () => {
  const root = mount(<RobotsPanel fleets={fleets} />);
  const robotElements = root.find(RobotItem);
  expect(robotElements.length).toBe(robots.length);
  root.unmount();
});

it('umount on exit', () => {
  const root = mount(<RobotsPanel fleets={fleets} />);
  const robotElement = root.find(RobotItem).at(0);

  // expansion details should be unmounted at the start
  expect(robotElement.find(ExpansionPanelDetails).length).toBe(0);

  robotElement.find(ExpansionPanelSummary).simulate('click');

  // now the details should be mounted
  expect(robotElement.update().find(ExpansionPanelDetails).length).toBe(1);

  root.unmount();
});

it('fires robot click event', () => {
  let clicked = false;
  const root = mount(<RobotsPanel fleets={fleets} onRobotClick={() => (clicked = true)} />);
  const robotElement = root.find(RobotItem).at(0);
  robotElement.simulate('click');
  expect(clicked).toBe(true);
  root.unmount();
});
