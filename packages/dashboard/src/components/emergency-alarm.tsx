import { IconButton } from '@material-ui/core';
import React from 'react';
import { Alerts } from '../util/alerts';
import NotificationsActiveIcon from '@material-ui/icons/NotificationsActive';

interface EmergencyAlarmProps {
  onTurnOn?: () => void;
  onTurnOff?: () => void;
}

export const EmergencyAlarm = React.memo((props: EmergencyAlarmProps) => {
  const { onTurnOn, onTurnOff } = props;
  const [alarm, setAlarm] = React.useState(false);

  const handleAlarm = (): void => {
    if (alarm) {
      Alerts.verification({
        confirmCallback: () => {
          setAlarm(false);
          onTurnOff && onTurnOff();
        },
        body: `You're about to turn off the alarm! The robots will resume their tasks.`,
      });
    } else {
      Alerts.verification({
        confirmCallback: () => {
          setAlarm(true);
          onTurnOn && onTurnOn();
        },
        body: `You're about to fire an alarm! The robots will head to their nearest holding points. Once you accept this there is no turning back.`,
      });
    }
  };

  return (
    <IconButton id="alarm-btn" color={alarm ? 'secondary' : 'inherit'} onClick={handleAlarm}>
      <NotificationsActiveIcon />
    </IconButton>
  );
});
