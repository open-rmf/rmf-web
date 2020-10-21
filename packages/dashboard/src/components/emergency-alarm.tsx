import { IconButton } from '@material-ui/core';
import React from 'react';
import NotificationsActiveIcon from '@material-ui/icons/NotificationsActive';

interface EmergencyAlarmProps {
  isActive: boolean | null;
}

export const EmergencyAlarm = React.memo((props: EmergencyAlarmProps) => {
  const { isActive } = props;
  return (
    <>
      {isActive && (
        <IconButton id="alarm-btn" color={'secondary'}>
          <NotificationsActiveIcon />
        </IconButton>
      )}
    </>
  );
});
