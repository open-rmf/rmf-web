import { Meta, Story } from '@storybook/react';
import React from 'react';
import { AlertContent, AlertDialog, DialogAlertProps } from './alert-dialog';

const buildAlertDialogContent = (): AlertContent[] => {
  return [
    {
      title: 'ID',
      value: 'testAlertID',
    },
    {
      title: 'Error logs',
      value: '1/1/1970 00:00:00 - error',
    },
    {
      title: 'Logs',
      value: '1/1/1970 00:00:00 - completed',
    },
  ];
};
export default {
  title: 'Alert Dialog',
  component: AlertDialog,
} as Meta;

export const AlertDialogComponent: Story<DialogAlertProps> = () => {
  return (
    <AlertDialog
      dismiss={() => {}}
      acknowledge={() => {}}
      title={'Alert Dialog'}
      progress={1}
      alertContents={buildAlertDialogContent()}
      backgroundColor={'ffff'}
    ></AlertDialog>
  );
};
