import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TrashBinControlButtonGroup } from './trashbin-control-button-group';
import { makeStyles } from '@material-ui/core';

const useStyles = makeStyles((theme) => ({
  omnipanel: {
    backgroundColor: theme.palette.background.paper,
  },
}));

export default {
  title: 'Trashbin Control Button Group',
  component: TrashBinControlButtonGroup,
  argTypes: {
    fullWidth: { control: 'boolean' },
  },
} as Meta;

export const SimpleTrashBinControlButtonGroup: Story = (args) => (
  // add a background to visualize different theme colors properly
  <TrashBinControlButtonGroup className={useStyles().omnipanel} {...args} />
);
