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
      id: 1,
      time: getDate(),
      error: 'coke_ingestor not sending states',
      severity: 'High',
    },
    {
      id: 2,
      time: getDate(),
      error: 'Lift is on fire',
      severity: 'High',
    },
    {
      id: 3,
      time: getDate(),
      error: 'Trajectory conflict with robot B and robot C',
      severity: 'Medium',
    },
    {
      id: 4,
      time: getDate(),
      error: 'Trajectory conflict with robot B and robot C',
      severity: 'Medium',
    },
    {
      id: 5,
      time: getDate(),
      error: 'Trajectory conflict with robot B and robot C',
      severity: 'Medium',
    },
    {
      id: 6,
      time: getDate(),
      error: 'Robot D not moving for over 10 seconds',
      severity: 'Low',
    },
    {
      id: 7,
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
        onClose={() => setShowDialogBox(false)}
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
