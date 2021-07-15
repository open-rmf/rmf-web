import {
  Dialog,
  DialogContent,
  DialogTitle,
  makeStyles,
  Paper,
  Typography,
} from '@material-ui/core';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { ConfirmDialogActions, ConfirmDialogActionsProps } from './confirm-dialog-actions';

export default {
  title: 'Dialog Actions/Confirm',
  component: ConfirmDialogActions,
} as Meta;

const useStyles = makeStyles({
  button: {
    width: 80,
  },
});

export const Confirm: Story<ConfirmDialogActionsProps> = (args) => {
  const classes = useStyles();
  return (
    <Dialog open={true}>
      <DialogTitle>Example</DialogTitle>
      <DialogContent>
        <Paper variant="outlined" style={{ padding: 200 }}>
          <Typography>Dialog Content</Typography>
        </Paper>
      </DialogContent>
      <ConfirmDialogActions
        {...args}
        classes={classes}
        confirmAction={() => new Promise((res) => setTimeout(res, 500))}
      />
    </Dialog>
  );
};
