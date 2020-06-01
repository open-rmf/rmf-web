import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { createMount } from '@material-ui/core/test-utils';
import { ReactWrapper } from 'enzyme';
import React from 'react';
import SingleHingeDoor from '../door-single-hinge';
import SingleSlideDoor from '../door-single-slide';
import DefaultDoor from '../door-default';

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
  const handleClick = () => {
    clicked = true;
  };
  const wrapper = mount(
    <svg>
      <DefaultDoor
        v1={[mainDoor.v1_x, mainDoor.v1_y]}
        v2={[mainDoor.v2_x, mainDoor.v2_y]}
        onClick={handleClick}
      />
    </svg>,
  );
  wrapper
    .find(DefaultDoor)
    .at(0)
    .find('g')
    .simulate('click');
  expect(clicked).toBe(true);

  wrapper.unmount();
});

describe('Checks assignation of styles on different states of SingleHingeDoor', () => {
  const getSingleDoorSVGLine = (wrapper: ReactWrapper) => {
    return wrapper
      .find(DefaultDoor)
      .at(0)
      .find('line')
      .at(0);
  };

  test('Style of SingleHingeDoor on Open', () => {
    const wrapper = mount(
      <svg>
        <SingleHingeDoor
          v1={[mainDoor.v1_x, mainDoor.v1_y]}
          v2={[mainDoor.v2_x, mainDoor.v2_y]}
          door={mainDoor}
          currentMode={RomiCore.DoorMode.MODE_OPEN}
        />
      </svg>,
    );
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorOpen)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of SingleHingeDoor on Close', () => {
    const wrapper = mount(
      <svg>
        <SingleHingeDoor
          v1={[mainDoor.v1_x, mainDoor.v1_y]}
          v2={[mainDoor.v2_x, mainDoor.v2_y]}
          door={mainDoor}
          currentMode={RomiCore.DoorMode.MODE_CLOSED}
        />
      </svg>,
    );
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorClose)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of SingleHingeDoor on Process', () => {
    const wrapper = mount(
      <svg>
        <SingleSlideDoor
          v1={[mainDoor.v1_x, mainDoor.v1_y]}
          v2={[mainDoor.v2_x, mainDoor.v2_y]}
          door={mainDoor}
          currentMode={RomiCore.DoorMode.MODE_MOVING}
        />
      </svg>,
    );
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorProcess)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });
});
