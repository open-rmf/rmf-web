import { IconButton } from '@mui/material';
import React from 'react';
import NotificationsActiveIcon from '@mui/icons-material/NotificationsActive';

export interface EmergencyAlarmProps {
  isActive: boolean | null;
}

/**
 * TODO: This component represents the emergency alarm. It is kept as a separate component in case
 * more logic needs to be added in the future, such as turning the alarm on and off. This component
 * is on hold until an appropriate backend is available to manage the alarm.
 *
 */
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
