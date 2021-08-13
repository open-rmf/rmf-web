import { Meta, Story } from '@storybook/react';
import React from 'react';
import { DoorAccordion } from './door-accordion';
import { DoorMode, DoorMotion, DoorType } from '../utils';
import { makeDoor, makeDoorState } from '../test-utils.spec';
import { doors, doorStates } from '../test-utils.spec';

export default {
  title: 'Door Accordion',
  component: DoorAccordion,
  argTypes: {
    door: {
      table: {
        disable: true,
      },
    },
    doorState: {
      table: {
        disable: true,
      },
    },
    name: {
      control: {
        type: 'text',
      },
    },
    door_type: {
      control: {
        type: 'select',
        options: Object.values(DoorType).filter((k) => typeof k === 'string'),
      },
    },
    motion_direction: {
      control: {
        type: 'select',
        options: Object.values(DoorMotion).filter((k) => typeof k === 'string'),
      },
    },
    current_mode: {
      control: {
        type: 'select',
        options: Object.values(DoorMode).filter((k) => typeof k === 'string'),
      },
    },
  },
} as Meta;

export const Basic: Story = ({
  name,
  door_type,
  motion_direction,
  motion_range,
  current_mode,
  ...args
}) => {
  const door = React.useMemo(
    () =>
      makeDoor({
        name: name,
        door_type: DoorType[door_type as keyof typeof DoorType],
        motion_direction: DoorMotion[motion_direction as keyof typeof DoorMotion],
        motion_range,
      }),
    [name, door_type, motion_direction, motion_range],
  );
  const state = React.useMemo(
    () =>
      makeDoorState({
        door_name: name,
        current_mode: { value: DoorMode[current_mode as keyof typeof DoorMode] },
      }),
    [name, current_mode],
  );
  return <DoorAccordion door={door} doorState={state} {...args} />;
};
Basic.args = {
  name: 'Basic Door',
  door_type: 'SingleSwing' as keyof typeof DoorType,
  motion_direction: 'Clockwise' as keyof typeof DoorMotion,
  motion_range: 1.571,
  current_mode: 'Open' as keyof typeof DoorMode,
};

export const DoorsPanel: Story = (args) => {
  return (
    <>
      {doors.map((door) => (
        <DoorAccordion key={door.name} door={door} doorState={doorStates[door.name]} {...args} />
      ))}
    </>
  );
};
