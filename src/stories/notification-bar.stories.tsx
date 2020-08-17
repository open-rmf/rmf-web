import React from 'react';

import NotificationBarStory from './BaseComponents/notificationBar-component';

export default {
  title: 'Notification Bar',
};

export const error = () => <NotificationBarStory message={'Error message'} type={'error'} />;

export const warning = () => <NotificationBarStory message={'Warning message'} type={'warning'} />;

export const info = () => <NotificationBarStory message={'Info message'} type={'info'} />;

export const success = () => <NotificationBarStory message={'Success message'} type={'success'} />;
