import React from 'react';

import NotificationBarStory from './BaseComponents/notificationBar';

export default {
    title: 'Notification Bar',
 };


export const Error = () => (
         <NotificationBarStory message={'Error message'} type={"error"}/>
 )

export const Warning = () => (
    <NotificationBarStory message={'Warning message'} type={"warning"}/>
)

export const Info = () => (
    <NotificationBarStory message={'Info message'} type={"info"}/>
)

export const Success = () => (
    <NotificationBarStory message={'Success message'} type={"success"}/>
)