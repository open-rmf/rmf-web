import React, { useState, useEffect } from 'react';
import Alert, { AlertProps } from '@material-ui/lab/Alert';
import { Snackbar } from '@material-ui/core';

interface NotificationBarProps {
  message: string;
  type: AlertProps['severity']; // error | warning | info | success
  time?: string; // milliseconds
}

const NotificationBar = (props: NotificationBarProps) => {
  const [showAlert, setShowAlert] = useState(false);
  const { message, type } = props;

  useEffect(() => {
    setShowAlert(true);
  }, [message]);

  return (
    <Snackbar open={showAlert} autoHideDuration={3000} onClose={() => setShowAlert(false)}>
      <Alert onClose={() => setShowAlert(false)} severity={type ? type : 'error'}>
        {message}
      </Alert>
    </Snackbar>
  );
};

export default NotificationBar;
