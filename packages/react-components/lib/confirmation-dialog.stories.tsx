import { Button, Paper, Typography } from '@mui/material';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { ConfirmationDialog, ConfirmationDialogProps } from './confirmation-dialog';

export default {
  title: 'Dialog/Confirmation Dialog',
  component: ConfirmationDialog,
  argTypes: {
    onClose: { actions: 'close' },
  },
} as Meta;

export const Default: Story<ConfirmationDialogProps> = (args) => {
  return (
    <ConfirmationDialog
      {...args}
      open={true}
      toolbar={
        <Button variant="contained" color="primary">
          Action
        </Button>
      }
    >
      <Paper variant="outlined" style={{ padding: 200 }}>
        <Typography>Content</Typography>
      </Paper>
    </ConfirmationDialog>
  );
};

Default.storyName = 'Confirmation Dialog';
