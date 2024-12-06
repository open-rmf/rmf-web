import { Meta, StoryObj } from '@storybook/react';

import { DeliveryTaskForm, makeDefaultDeliveryTaskDescription } from './delivery';

export default {
  title: 'Tasks/DeliveryTaskForm',
  component: DeliveryTaskForm,
} satisfies Meta;

type Story = StoryObj<typeof DeliveryTaskForm>;

export const Default: Story = {
  args: {
    taskDesc: makeDefaultDeliveryTaskDescription(),
    pickupPoints: { pickup_1: 'handler_1', pickup_2: 'handler_2' },
    dropoffPoints: { dropoff_1: 'handler_3', dropoff_2: 'handler_4' },
    onChange: () => {},
    onValidate: () => {},
  },
};
