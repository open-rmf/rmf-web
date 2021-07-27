import { Snackbar } from '@material-ui/core';
import Alert, { AlertProps } from '@material-ui/lab/Alert';
import React, { useEffect } from 'react';
import * as RmfModels from 'rmf-models';

export const iniCharger: RmfModels.ChargerRequest = {
  charger_name: '',
  fleet_name: '',
  robot_name: '',
  start_timeout: { sec: 0, nanosec: 0 },
  request_id: '',
};

export interface AlertSnackBarProps {
  message: string | undefined | null;
  charger: RmfModels.ChargerRequest;
  type: AlertProps['severity']; // error | warning | info | success
  showAlert: boolean;
  time?: string; // milliseconds
  onMessageClose?: () => void;
  onShowAlert?: () => void;
}

const AlertSnackBar = (props: AlertSnackBarProps) => {
  const { message, type, charger, showAlert, onShowAlert, onMessageClose } = props;

  useEffect(() => {
    if (!!message && charger.robot_name.length > 0) {
      onShowAlert && onShowAlert();
    }
  }, [message, charger.robot_name.length, onShowAlert]);

  return (
    <Snackbar
      style={{ height: '100%' }}
      anchorOrigin={{ vertical: 'top', horizontal: 'center' }}
      open={showAlert}
      autoHideDuration={6000}
      onClose={() => onMessageClose && onMessageClose()}
    >
      <Alert onClose={() => onMessageClose && onMessageClose()} severity={type ? type : 'error'}>
        {message}
      </Alert>
    </Snackbar>
  );
};

export default AlertSnackBar;
