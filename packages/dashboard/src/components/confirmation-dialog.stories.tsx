import { Button, Paper, Typography } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';

import { ConfirmationDialog } from './confirmation-dialog';

export default {
  title: 'Dialog/Confirmation Dialog',
  component: ConfirmationDialog,
  argTypes: {
    onClose: { actions: 'close' },
  },
} satisfies Meta;

type Story = StoryObj<typeof ConfirmationDialog>;

export const Default: Story = {
  storyName: 'Confirmation Dialog',
  render: (args) => (
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
  ),
};
