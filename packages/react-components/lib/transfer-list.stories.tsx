import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TransferList, TransferListProps } from './transfer-list';

export default {
  title: 'Transfer List',
  component: TransferList,
  argTypes: {
    title: {
      defaultValue: 'Choices',
    },
    leftItems: {
      control: false,
    },
    rightItems: {
      control: false,
    },
  },
} as Meta;

export const TransferListStory: Story<TransferListProps> = (args) => {
  const [leftItems, setLeftItems] = React.useState(['one', 'two', 'three', 'four']);
  const [rightItems, setRightItems] = React.useState(['five', 'six', 'seven', 'eight']);
  return (
    <TransferList
      {...args}
      leftItems={leftItems}
      rightItems={rightItems}
      onTransfer={(left, right) => {
        setLeftItems(left);
        setRightItems(right);
      }}
    ></TransferList>
  );
};

TransferListStory.storyName = 'Transfer List';
