import { Snackbar, makeStyles, Typography, Button, Grid } from '@material-ui/core';
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

const useStyles = makeStyles((theme) => ({
  snackbar: {
    height: '100%',
  },
  cookieAlert: {
    '& .MuiAlert-icon': {
      fontSize: '2.5rem',
    },
    padding: '2rem',
    width: '1000px',
  },
  button: {
    color: theme.palette.warning.light,
    backgroundColor: 'rgb(102, 60, 0)',
    width: '100%',
    marginTop: '0.5rem',
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
      <Alert className={classes.cookieAlert} severity={type ? type : 'error'}>
        <Grid container direction="column" justify="center" alignItems="center">
          <Grid item>
            <Typography variant="h5">{message}</Typography>
          </Grid>
        </Grid>
        <Grid item>
          <Button
            className={classes.button}
            color="inherit"
            onClick={() => onMessageClose && onMessageClose()}
          >
            <Typography variant="h5">Ok</Typography>
          </Button>
        </Grid>
      </Alert>
    </Snackbar>
  );
};

export default AlertSnackBar;
