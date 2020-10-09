import { IconButton } from '@material-ui/core';
import React from 'react';
import { Alerts } from '../util/alerts';
import NotificationsActiveIcon from '@material-ui/icons/NotificationsActive';

interface EmergencyAlarmProps {
  onTurnOn?: () => void;
  onTurnOff?: () => void;
  isActive: boolean | null;
}

export const EmergencyAlarm = React.memo((props: EmergencyAlarmProps) => {
  const { onTurnOn, onTurnOff, isActive } = props;

  const handleAlarm = (): void => {
    if (isActive) {
      Alerts.verification({
        confirmCallback: () => {
          onTurnOff && onTurnOff();
        },
        body: `You're about to turn off the alarm! The robots will resume their tasks.`,
      });
    } else {
      Alerts.verification({
        confirmCallback: () => {
          onTurnOn && onTurnOn();
        },
        body: `You're about to fire an alarm! The robots will head to their nearest holding points. Once you accept this there is no turning back.`,
      });
    }
  };

  return (
    <IconButton id="alarm-btn" color={isActive ? 'secondary' : 'inherit'} onClick={handleAlarm}>
      <NotificationsActiveIcon />
    </IconButton>
  );
});
