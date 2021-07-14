import React from 'react';
import { SystemSummaryAlertProps, SystemSummaryAlert, Severity } from '../lib';
import { Meta, Story } from '@storybook/react';
import { format } from 'date-fns';

export default {
  title: 'Systems summary alert',
  component: SystemSummaryAlert,
} as Meta;

const systemSummaryAlert: SystemSummaryAlertProps = {
  notifications: [
    {
      id: 1,
      severity: Severity.High,
      time: format(new Date(), 'MM/dd/yyyy HH:mm'),
      error: 'hello world',
    },
  ],
  onNotificationsDismiss: () => {
    /**filler */
  },
};

const SimpleNotifications = (): JSX.Element => {
  const [allNotifications, setAllNotifications] = React.useState(systemSummaryAlert.notifications);

  const handleDismissNotification = (id: number) => {
    const filteredNotifications = allNotifications.filter((n) => n.id !== id);
    setAllNotifications(filteredNotifications);
  };

  return (
    <SystemSummaryAlert
      notifications={allNotifications}
      onNotificationsDismiss={handleDismissNotification}
    />
  );
};

export const SystemSummaryItemAlertStory: Story = (args) => (
  <React.Fragment>
    <SimpleNotifications {...args} />
    <SystemSummaryAlert
      notifications={[]}
      onNotificationsDismiss={systemSummaryAlert.onNotificationsDismiss}
      {...args}
    />
  </React.Fragment>
);
