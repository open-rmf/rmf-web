import React, { useState, useEffect } from 'react';
import Alert, { AlertProps } from '@material-ui/lab/Alert';
import { Snackbar } from '@material-ui/core';

export interface NotificationBarProps {
  message: string | undefined | null;
  type: AlertProps['severity']; // error | warning | info | success
  time?: string; // milliseconds
}

const NotificationBar = (props: NotificationBarProps) => {
  const [showAlert, setShowAlert] = useState(false);
  const { message, type } = props;

  useEffect(() => {
    if (!!message) {
      setShowAlert(true);
    }
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

export const NotificationBarContext = React.createContext<React.Dispatch<
  React.SetStateAction<NotificationBarProps | null>
> | null>(null);
