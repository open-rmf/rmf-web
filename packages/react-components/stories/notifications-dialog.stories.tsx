import React from 'react';
import { Button } from '@material-ui/core';
import { NotificationsDialog, Notification } from '../lib';
import { Meta, Story } from '@storybook/react';
import moment from 'moment';

export default {
  title: 'Notifications Dialogue',
  component: NotificationsDialog,
} as Meta;

interface NotificationDialogHandlerProps {
  notifications: Notification[];
}

// temp data for display
const getDate = () => {
  return moment().format('MMMM Do YYYY, h:mm:ss');
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
// end of temp data

const NotificationDialogHandler = (props: NotificationDialogHandlerProps): JSX.Element => {
  const { notifications } = props;
  const [showDialogBox, setShowDialogBox] = React.useState(false);

  return (
    <>
      <Button variant="contained" onClick={() => setShowDialogBox(true)}>
        Open Notificatins Dialog box
      </Button>
      <NotificationsDialog
        notifications={notifications}
        showNotificationsDialog={showDialogBox}
        setShowNotifications={() => setShowDialogBox(false)}
      />
    </>
  );
};

export const NotificationsDialogDisplay: Story = () => {
  return (
    <>
      <NotificationDialogHandler notifications={makeNotifications()} />
    </>
  );
};
