import React, { useState, useEffect, useContext } from 'react';
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
  const notificationDispatch = useContext(NotificationBarContext);

  useEffect(() => {
    if (!!message) {
      setShowAlert(true);
    }
  }, [message]);

  const onMessageClose = () => {
    setShowAlert(false);
    notificationDispatch &&
      notificationDispatch({
        message: undefined,
        type: undefined,
      });
  };

  return (
    <Snackbar open={showAlert} autoHideDuration={3000} onClose={() => onMessageClose()}>
      <Alert onClose={() => onMessageClose()} severity={type ? type : 'error'}>
        {message}
      </Alert>
    </Snackbar>
  );
};

export default NotificationBar;
