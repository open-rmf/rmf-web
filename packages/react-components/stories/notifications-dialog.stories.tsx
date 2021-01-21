import React from 'react';
import { Button } from '@material-ui/core';
import { NotificationsDialog, Notification } from '../lib';
import { Meta, Story } from '@storybook/react';

export default {
  title: 'Notifications Dialogue',
  component: NotificationsDialog,
} as Meta;

interface NotificationDialogHandlerProps {
  notifications: Notification[];
}

const getDate = () => {
  const date = new Date();
  const dateStr =
    ('00' + (date.getMonth() + 1)).slice(-2) +
    '/' +
    ('00' + date.getDate()).slice(-2) +
    '/' +
    date.getFullYear() +
    ' ' +
    ('00' + date.getHours()).slice(-2) +
    ':' +
    ('00' + date.getMinutes()).slice(-2) +
    ':' +
    ('00' + date.getSeconds()).slice(-2);
  return dateStr;
};

const makeNotifications = (): Notification[] => {
  return [
    {
      time: getDate(),
      error: 'coke_ingestor not sending states',
      severity: 'High',
    },
    {
      time: getDate(),
      error: 'Lift is on fire',
      severity: 'High',
    },
    {
      time: getDate(),
      error: 'Trajectory conflict with robot B and robot C',
      severity: 'Medium',
    },
    {
      time: getDate(),
      error: 'Trajectory conflict with robot B and robot C',
      severity: 'Medium',
    },
    {
      time: getDate(),
      error: 'Trajectory conflict with robot B and robot C',
      severity: 'Medium',
    },
    {
      time: getDate(),
      error: 'Robot D not moving for over 10 seconds',
      severity: 'Low',
    },
    {
      time: getDate(),
      error: 'Lift is offline',
      severity: 'Low',
    },
  ];
};

const NotificationDialogHandler = (props: NotificationDialogHandlerProps): JSX.Element => {
  const { notifications } = props;
  const [showDialogBox, setShowDialogBox] = React.useState(false);

  return (
    <>
      <Button variant="contained" onClick={() => setShowDialogBox(true)}>
        Open Dialog
      </Button>
      <NotificationsDialog
        notifications={notifications}
        showNotificationsDialog={showDialogBox}
        setShowNotifications={() => setShowDialogBox(false)}
      />
    </>
  );
};

export const LiftsPanel: Story = () => {
  return (
    <>
      <NotificationDialogHandler notifications={makeNotifications()} />
    </>
  );
};
