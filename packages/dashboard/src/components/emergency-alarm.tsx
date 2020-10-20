import { IconButton } from '@material-ui/core';
import React from 'react';
import NotificationsActiveIcon from '@material-ui/icons/NotificationsActive';
import { ConfirmAlertDialog } from './confirm-alert-dialog';

interface EmergencyAlarmProps {
  onTurnOn?: () => void;
  onTurnOff?: () => void;
  isActive: boolean | null;
}

export const EmergencyAlarm = React.memo((props: EmergencyAlarmProps) => {
  const { onTurnOn, onTurnOff, isActive } = props;
  const [openConfirmationDialog, setOpenConfirmationDialog] = React.useState(false);
  const handleEmergencyAlarmClick = () => {
    setOpenConfirmationDialog(true);
  };

  const closeDialog = React.useCallback(() => setOpenConfirmationDialog(false), []);

  return (
    <>
      {isActive ? (
        <ConfirmAlertDialog
          open={openConfirmationDialog}
          close={closeDialog}
          content={`You're about to turn off the alarm! The robots will resume their tasks.`}
          confirmCallback={() => onTurnOff && onTurnOff()}
          showIcon={true}
        />
      ) : (
        <ConfirmAlertDialog
          open={openConfirmationDialog}
          close={closeDialog}
          content={`You're about to fire an alarm! The robots will head to their nearest holding points. Once you accept this there is no turning back.`}
          confirmCallback={() => onTurnOn && onTurnOn()}
          showIcon={true}
        />
      )}

      <IconButton
        id="alarm-btn"
        color={isActive ? 'secondary' : 'inherit'}
        onClick={handleEmergencyAlarmClick}
      >
        <NotificationsActiveIcon />
      </IconButton>
    </>
  );
});
