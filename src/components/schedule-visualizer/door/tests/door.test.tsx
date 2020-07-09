import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { createMount } from '@material-ui/core/test-utils';
import { ReactWrapper, shallow } from 'enzyme';
import React from 'react';
import SingleHingeDoor from '../door-single-hinge';
import SingleSlideDoor from '../door-single-slide';
import DefaultDoor from '../door-default';
import DoubleHingeDoor from '../door-double-hinge';
import { DoorProps, getDoorStyle } from '../door';
import DoubleSlideDoor from '../door-double-slide';
import renderer from 'react-test-renderer';

const mount = createMount();

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

const getDoorComponent = (
  Component: (props: DoorProps) => React.ReactElement,
  door: RomiCore.Door,
  currentMode: number,
): JSX.Element => {
  return (
    <svg>
      <Component
        v1={[mainDoor.v1_x, mainDoor.v1_y]}
        v2={[mainDoor.v2_x, mainDoor.v2_y]}
        door={door}
        currentMode={currentMode}
      />
    </svg>
  );
};

const buildDoorWrapper = (
  Component: (props: DoorProps) => React.ReactElement,
  door: RomiCore.Door,
  currentMode: number,
) => {
  return mount(getDoorComponent(Component, door, currentMode));
};

const getSingleDoorSVGLine = (wrapper: ReactWrapper) => {
  return wrapper
    .find(DefaultDoor)
    .at(0)
    .find('line')
    .at(0);
};

const checkOpenStyle = (
  Component: (props: DoorProps) => React.ReactElement,
  door: RomiCore.Door,
  currentMode: number,
) => {
  const wrapper = buildDoorWrapper(Component, door, currentMode);
  const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
  return singleDoorSVGLine.hasClass(/(makeStyles)-(doorOpen)-(\d+)/);
};

const checkClosedStyle = (
  Component: (props: DoorProps) => React.ReactElement,
  door: RomiCore.Door,
  currentMode: number,
) => {
  const wrapper = buildDoorWrapper(Component, door, currentMode);
  const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
  return singleDoorSVGLine.hasClass(/(makeStyles)-(doorClose)-(\d+)/);
};

const checkProcessStyle = (
  Component: (props: DoorProps) => React.ReactElement,
  door: RomiCore.Door,
  currentMode: number,
) => {
  const wrapper = buildDoorWrapper(Component, door, currentMode);
  const singleDoorSVGLine = getSingleDoorSVGLine(wrapper);
  return singleDoorSVGLine.hasClass(/(makeStyles)-(doorProcess)-(\d+)/);
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

test('Doors renders correctly', () => {
  expect(
    renderer
      .create(getDoorComponent(SingleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN))
      .toJSON(),
  ).toMatchSnapshot();

  expect(
    renderer
      .create(getDoorComponent(DoubleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN))
      .toJSON(),
  ).toMatchSnapshot();

  expect(
    renderer
      .create(getDoorComponent(SingleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN))
      .toJSON(),
  ).toMatchSnapshot();

  expect(
    renderer
      .create(getDoorComponent(DoubleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN))
      .toJSON(),
  ).toMatchSnapshot();
});

describe('Checks assignation of styles on different states', () => {
  test('Style of SingleHingeDoor', () => {
    expect(checkOpenStyle(SingleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN)).toBe(true);
    expect(checkClosedStyle(SingleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_CLOSED)).toBe(true);
    expect(checkProcessStyle(SingleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_MOVING)).toBe(true);
  });

  test('Style of SingleSlideDoor', () => {
    expect(checkOpenStyle(SingleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN)).toBe(true);
    expect(checkClosedStyle(SingleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_CLOSED)).toBe(true);
    expect(checkProcessStyle(SingleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_MOVING)).toBe(true);
  });

  test('Style of DobleHingeDoors', () => {
    expect(checkOpenStyle(DoubleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN)).toBe(true);
    expect(checkClosedStyle(DoubleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_CLOSED)).toBe(true);
    expect(checkProcessStyle(DoubleHingeDoor, mainDoor, RomiCore.DoorMode.MODE_MOVING)).toBe(true);
  });

  test('Style of DoubleSlideDoor', () => {
    expect(checkOpenStyle(DoubleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_OPEN)).toBe(true);
    expect(checkClosedStyle(DoubleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_CLOSED)).toBe(true);
    expect(checkProcessStyle(DoubleSlideDoor, mainDoor, RomiCore.DoorMode.MODE_MOVING)).toBe(true);
  });
});
