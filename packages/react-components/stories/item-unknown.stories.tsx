import React from 'react';
import { Meta, Story } from '@storybook/react';
import { ItemUnknown, SimpleInfo } from '../lib';
import { makeStyles } from '@material-ui/core';

export default {
  title: 'Item Unknown',
  component: ItemUnknown,
} as Meta;

export const ItemUnknownPanel: Story = (args) => {
  // override style with userSelect disabled
  const useStyles = makeStyles(() => ({
    container: {
      display: 'table',
      borderCollapse: 'collapse',
      width: '100%',
      overflowX: 'auto',
      userSelect: 'none',
    },
  }));
  const classes = useStyles();

  function TestComponent() {
    const data = [
      { name: 'String', value: 'This is a string' },
      { name: 'Number', value: 3 },
      {
        name: 'Array',
        value: ['one', 'two', 'three'],
      },
    ];
    return <SimpleInfo infoData={data} overrideStyle={classes} />;
  }

  return (
    <>
      <ItemUnknown errorMsg={'This is an error message'} showError={true} {...args}>
        <TestComponent />
      </ItemUnknown>
    </>
  );
};
