import { createMount } from '@material-ui/core/test-utils';
import { Spinner } from '../spinner';
import Door, { SingleHingeDoor, DoorMode } from './door';
import React from 'react';

const mainDoor = {
  name: 'main_door',
  v1_x: 8.2,
  v1_y: -5.5,
  v2_x: 7.85,
  v2_y: -6.2,
  door_type: 2,
  motion_range: -1.571,
  motion_direction: 1,
};

const mount = createMount();

test('Trigger click event', async () => {
  let clicked = false;

  const wrapper = mount(
    <svg>
      <Door door={mainDoor} onClick={() => (clicked = true)} />
    </svg>,
  );

  wrapper
    .find(Door)
    .at(0)
    .simulate('click');
  expect(clicked).toBe(true);

  wrapper.unmount();
});

test('Checks spinner activation when the door its opening or closing', async () => {
  const wrapper = mount(
    <svg>
      <Door door={mainDoor} currentMode={DoorMode.PROCESS} />
    </svg>,
  );
  expect(
    wrapper
      .find(Door)
      .at(0)
      .find(Spinner)
      .exists(),
  ).toBe(true);
  wrapper.unmount();
});

describe('Checks assignation of styles on different states of SingleHingeDoor', () => {
  test('Style of SingleHingeDoor on Open', () => {
    const wrapper = mount(
      <svg>
        <SingleHingeDoor door={mainDoor} currentMode={DoorMode.OPEN} />
      </svg>,
    );
    const singleDoorSVGLine = wrapper
      .find(SingleHingeDoor)
      .at(0)
      .find('line');
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorOpen)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of SingleHingeDoor on Close', () => {
    const wrapper = mount(
      <svg>
        <SingleHingeDoor door={mainDoor} currentMode={DoorMode.CLOSE} />
      </svg>,
    );
    const singleDoorSVGLine = wrapper
      .find(SingleHingeDoor)
      .at(0)
      .find('line');
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorClose)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of SingleHingeDoor on Process', () => {
    const wrapper = mount(
      <svg>
        <SingleHingeDoor door={mainDoor} currentMode={DoorMode.PROCESS} />
      </svg>,
    );
    const singleDoorSVGLine = wrapper
      .find(SingleHingeDoor)
      .at(0)
      .find('line');
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorProcess)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });
});
