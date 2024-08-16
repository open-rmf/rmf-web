import { Paper } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';
import React from 'react';

import { TransferList } from './transfer-list';

export default {
  title: 'Transfer List',
  component: TransferList,
  argTypes: {
    leftItems: {
      control: false,
    },
    rightItems: {
      control: false,
    },
    leftTitle: {
      control: {
        type: 'text',
      },
      defaultValue: 'Choices',
    },
    rightTitle: {
      control: {
        type: 'text',
      },
      defaultValue: 'Choices',
    },
  },
} satisfies Meta;

type Story = StoryObj<typeof TransferList>;

const numbers: Record<string, number> = {
  one: 1,
  two: 2,
  three: 3,
  four: 4,
  five: 5,
  six: 6,
  seven: 7,
  eight: 8,
};

export const TransferListStory: Story = {
  storyName: 'Transfer List',
  render: (args) => {
    const [leftItems, setLeftItems] = React.useState(['one', 'two', 'three', 'four']);
    const [rightItems, setRightItems] = React.useState(['five', 'six', 'seven', 'eight']);
    return (
      <Paper style={{ width: '100%', height: 250 }}>
        <TransferList
          {...args}
          leftItems={leftItems}
          rightItems={rightItems}
          onTransfer={(left, right) => {
            left.sort((a, b) => numbers[a] - numbers[b]);
            setLeftItems(left);
            right.sort((a, b) => numbers[a] - numbers[b]);
            setRightItems(right);
          }}
        ></TransferList>
      </Paper>
    );
  },
};
