import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import buildingMap from '../../mock/data/building-map';
import fakeDoorStates from '../../mock/data/door-states';
import FakeTransport from '../../mock/fake-transport';
import DoorItem from '../door-item';
import DoorsPanel from '../doors-panel';

const mount = createMount();

let map: RomiCore.BuildingMap;
let doorStates: Record<string, RomiCore.DoorState>;
let doors: RomiCore.Door[];
let transport: FakeTransport;

beforeEach(async () => {
  map = await buildingMap();
  doorStates = fakeDoorStates();
  doors = map.levels.flatMap(l => l.doors);
  transport = new FakeTransport();
});

it('renders doors', () => {
  const root = mount(<DoorsPanel doorStates={doorStates} doors={doors} />);
  const doorElements = root.find(DoorItem);
  expect(doorElements.length).toBe(doors.length);
  root.unmount();
});

it('publish request on open click', () => {
  const publishMock = jest.fn();
  jest.spyOn(transport, 'createPublisher').mockImplementation(() => {
    return {
      publish: publishMock,
    };
  });
  const root = mount(<DoorsPanel doorStates={doorStates} doors={doors} transport={transport} />);
  const doorElement = root.find(DoorItem).at(0);

  const openButton = doorElement.findWhere(x => x.name() === 'button' && x.text() === 'Open');
  openButton.simulate('click');

  expect(publishMock).toHaveBeenCalledTimes(1);
  root.unmount();
});

it('publish request on close click', () => {
  const publishMock = jest.fn();
  jest.spyOn(transport, 'createPublisher').mockImplementation(() => {
    return {
      publish: publishMock,
    };
  });
  const root = mount(<DoorsPanel doorStates={doorStates} doors={doors} transport={transport} />);
  const doorElement = root.find(DoorItem).at(0);

  const openButton = doorElement.findWhere(x => x.name() === 'button' && x.text() === 'Close');
  openButton.simulate('click');

  expect(publishMock).toHaveBeenCalledTimes(1);
  root.unmount();
});

it('fires on door click event', () => {
  let clicked = false;
  const root = mount(
    <DoorsPanel doorStates={doorStates} doors={doors} onDoorClick={() => (clicked = true)} />,
  );
  const doorElement = root.find(DoorItem).at(0);
  doorElement.simulate('click');
  expect(clicked).toBe(true);
  root.unmount();
});
