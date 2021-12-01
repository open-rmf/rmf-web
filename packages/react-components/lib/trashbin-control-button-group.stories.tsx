import { Meta, Story } from '@storybook/react';
import React from 'react';
import {
  TrashBinControlButtonGroup,
  TrashBinControlButtonGroupProps,
} from './trashbin-control-button-group';
import { styled } from '@mui/material';

const classes = {
  omnipanel: 'trashbin-control-root',
};

const TrashBinControlButtonGroupRoot = styled((props: TrashBinControlButtonGroupProps) => (
  <TrashBinControlButtonGroup {...props} />
))(({ theme }) => ({
  [`&.${classes.omnipanel}`]: {
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
  <TrashBinControlButtonGroupRoot className={classes.omnipanel} {...args} />
);
