import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { DoorItem } from '..';

export default {
  title: 'Doors',
  component: DoorItem,
  parameters: { actions: { argTypesRegex: '^on.*' } },
} as Meta;

const doors = [
  {
    name: 'main_door',
    v1_x: 8.2,
    v1_y: -5.5,
    v2_x: 7.85,
    v2_y: -6.2,
    door_type: 2,
    motion_range: -1.571,
    motion_direction: 1,
  },
  {
    name: 'hardware_door',
    v1_x: 4.9,
    v1_y: -4,
    v2_x: 4.4,
    v2_y: -5,
    door_type: 1,
    motion_range: 1.571,
    motion_direction: -1,
  },
  {
    name: 'coe_door',
    v1_x: 19.5,
    v1_y: -10.8,
    v2_x: 19.5,
    v2_y: -9.9,
    door_type: 1,
    motion_range: 1.571,
    motion_direction: 1,
  },
  {
    name: 'exit_door',
    v1_x: 12.2,
    v1_y: -2.7,
    v2_x: 14.1,
    v2_y: -2.7,
    door_type: 1,
    motion_range: -1.571,
    motion_direction: 1,
  },
];

const doorStates = {
  coe_door: {
    door_name: 'coe_door',
    current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
    door_time: { sec: 0, nanosec: 0 },
  },
  hardware_door: {
    door_name: 'hardware_door',
    current_mode: { value: RomiCore.DoorMode.MODE_CLOSED },
    door_time: { sec: 0, nanosec: 0 },
  },
  exit_door: {
    door_name: 'exit_door',
    current_mode: { value: RomiCore.DoorMode.MODE_MOVING },
    door_time: { sec: 0, nanosec: 0 },
  },
};

export const DoorsPanel: Story = (args) => {
  return (
    <>
      {doors.map((door) => (
        <DoorItem key={door.name} door={door} doorState={doorStates[door.name]} {...args} />
      ))}
    </>
  );
};
