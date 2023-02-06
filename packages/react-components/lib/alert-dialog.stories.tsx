import { Meta, Story } from '@storybook/react';
import React from 'react';
import { AlertContent, AlertDialog, DialogAlertProps } from './alert-dialog';

const buildDialogContent = (): AlertContent[] => {
  return [
    {
      title: 'Robot Name',
      value: 'Tiny1',
    },
    {
      title: 'Location',
      value: 'L1',
    },
    {
      title: 'Message',
      value: 'Robot has arrived at its destination ',
    },
  ];
};
export default {
  title: 'Alert Dialog',
  component: AlertDialog,
} as Meta;

export const AlertDialogComponent: Story<DialogAlertProps> = () => {
  const [show, setShow] = React.useState(true);
  return (
    <AlertDialog
      stopShowing={() => setShow(false)}
      dialogTitle={'Robot State'}
      progress={90}
      alertContents={buildDialogContent()}
      backgroundColor={'ffff'}
      show={show}
    ></AlertDialog>
  );
};
