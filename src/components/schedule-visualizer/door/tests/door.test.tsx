import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { createMount } from '@material-ui/core/test-utils';
import { ReactWrapper } from 'enzyme';
import React from 'react';
import SingleHingeDoor from '../door-single-hinge';
import SingleSlideDoor from '../door-single-slide';
import DefaultDoor from '../door-default';
import DoubleHingeDoor from '../door-double-hinge';
import { DoorProps, getDoorStyle } from '../door';
import DoubleSlideDoor from '../door-double-slide';
import doorStyles from '../door-style';

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

const buildDoorWrapper = (
  Component: (props: DoorProps) => React.ReactElement,
  mainDoor: RomiCore.Door,
  currentMode: number,
) => {
  return mount(
    <svg>
      <Component
        v1={[mainDoor.v1_x, mainDoor.v1_y]}
        v2={[mainDoor.v2_x, mainDoor.v2_y]}
        door={mainDoor}
        currentMode={currentMode}
      />
    </svg>,
  );
};

const mount = createMount();

const getSingleDoorSVGLine = (wrapper: ReactWrapper) => {
  return wrapper
    .find(DefaultDoor)
    .at(0)
    .find('line')
    .at(0);
};

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

test('Get correct door style', () => {
  const doorStyle = {
    doorOpen: 'doorOpenStyle',
    doorClose: 'doorCloseStyle',
    doorProcess: 'doorProcessStyle',
  };
  const open = getDoorStyle(doorStyle, RomiCore.DoorMode.MODE_OPEN);
  expect(open).toEqual(doorStyle.doorOpen);
  const closed = getDoorStyle(doorStyle, RomiCore.DoorMode.MODE_CLOSED);
  expect(closed).toEqual(doorStyle.doorClose);
  const moving = getDoorStyle(doorStyle, RomiCore.DoorMode.MODE_MOVING);
  expect(moving).toEqual(doorStyle.doorProcess);
});

describe('Checks assignation of styles on different states of SingleHingeDoor', () => {
  test('Style of SingleHingeDoor on Open', () => {
    const wrapper = buildDoorWrapper(SingleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorOpen)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of SingleHingeDoor on Close', () => {
    const wrapper = buildDoorWrapper(SingleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_CLOSED);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorClose)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of SingleHingeDoor on Process', () => {
    const wrapper = buildDoorWrapper(SingleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_MOVING);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorProcess)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });
});

describe('Checks assignation of styles on different states', () => {
  test('Style of SingleSlideDoor on Open', () => {
    const wrapper = buildDoorWrapper(SingleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorOpen)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of SingleSlideDoor on Close', () => {
    const wrapper = buildDoorWrapper(SingleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_CLOSED);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorClose)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of SingleSlideDoor on Process', () => {
    const wrapper = buildDoorWrapper(SingleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_MOVING);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorProcess)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });
});

describe('Checks assignation of styles on DobleHingeDoors', () => {
  const getSingleDoorSVGLine = (wrapper: ReactWrapper) => {
    return wrapper
      .find(DefaultDoor)
      .at(0)
      .find('line')
      .at(0);
  };

  test('Style of DoubleHingeDoors on Open', () => {
    const wrapper = buildDoorWrapper(DoubleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorOpen)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of DoubleHingeDoors on Close', () => {
    const wrapper = buildDoorWrapper(DoubleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_CLOSED);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorClose)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of DoubleHingeDoors on Process', () => {
    const wrapper = buildDoorWrapper(DoubleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_MOVING);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorProcess)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });
});

describe('Checks assignation of styles on DoubleSlideDoor', () => {
  const getSingleDoorSVGLine = (wrapper: ReactWrapper) => {
    return wrapper
      .find(DefaultDoor)
      .at(0)
      .find('line')
      .at(0);
  };

  test('Style of DoubleSlideDoor on Open', () => {
    const wrapper = buildDoorWrapper(DoubleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorOpen)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of DoubleSlideDoor on Close', () => {
    const wrapper = buildDoorWrapper(DoubleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_CLOSED);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorClose)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });

  test('Style of DoubleSlideDoor on Process', () => {
    const wrapper = buildDoorWrapper(DoubleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_MOVING);
    const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
    expect(singleDoorSVGLine.hasClass(/(makeStyles)-(doorProcess)-(\d+)/)).toBe(true);
    wrapper.unmount();
  });
});
