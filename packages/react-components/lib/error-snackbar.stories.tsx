import { Button } from '@mui/material';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { ErrorSnackbar, ErrorSnackbarProps } from './error-snackbar';

export default {
  title: 'Error Snackbar',
  component: ErrorSnackbar,
  argTypes: {
    message: {
      defaultValue: 'example error',
    },
  },
} as Meta;

export const Default: Story<ErrorSnackbarProps> = (args) => {
  const [open, setOpen] = React.useState(false);
  return (
    <div>
      <Button variant="contained" onClick={() => setOpen(true)}>
        Open
      </Button>
      <ErrorSnackbar {...args} open={open} onClose={() => setOpen(false)} />
    </div>
  );
};

Default.storyName = 'Error Snackbar';
