import { Snackbar, makeStyles, Typography, IconButton } from '@material-ui/core';
import Alert, { AlertProps } from '@material-ui/lab/Alert';
import React, { useEffect } from 'react';
import * as RmfModels from 'rmf-models';
import CloseIcon from '@material-ui/icons/Close';

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

const useStyles = makeStyles((theme) => ({
  snackbar: {
    height: '100%',
  },
  cookieAlert: {
    '& .MuiAlert-icon': {
      fontSize: '4rem',
    },
    padding: '2rem',
    width: '1000px',
  },
  closeIcon: {
    fontSize: '4rem',
  },
}));

const AlertSnackBar = (props: AlertSnackBarProps) => {
  const { message, type, charger, showAlert, onShowAlert, onMessageClose } = props;
  const classes = useStyles();

  useEffect(() => {
    if (!!message && charger.robot_name.length > 0) {
      onShowAlert && onShowAlert();
    }
  }, [message, charger.robot_name.length, onShowAlert]);

  return (
    <Snackbar
      className={classes.snackbar}
      anchorOrigin={{ vertical: 'top', horizontal: 'center' }}
      open={showAlert}
      autoHideDuration={null}
      onClose={() => onMessageClose && onMessageClose()}
    >
      <Alert
        className={classes.cookieAlert}
        severity={type ? type : 'error'}
        action={
          <IconButton color="inherit" onClick={() => onMessageClose && onMessageClose()}>
            <CloseIcon fontSize="large" />
          </IconButton>
        }
      >
        <Typography variant="h5">{message}</Typography>
      </Alert>
    </Snackbar>
  );
};

export default AlertSnackBar;
