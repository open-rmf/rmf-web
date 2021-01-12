import React from 'react';
import { Meta, Story } from '@storybook/react';
import { ItemUnknown, SimpleInfo } from '../lib';

export default {
  title: 'Item Unknown',
  component: ItemUnknown,
} as Meta;

export const ItemUnknownPanel: Story = (args) => {
  function TestComponent() {
    const data = [
      { name: 'String', value: 'This is a string' },
      { name: 'Number', value: 3 },
      {
        name: 'Array',
        value: ['one', 'two', 'three'],
      },
    ];
    return <SimpleInfo infoData={data} />;
  }

  return (
    <>
      <ItemUnknown errorMsg={'This is an error message'} showError={true} {...args}>
        <TestComponent />
      </ItemUnknown>
    </>
  );
};
