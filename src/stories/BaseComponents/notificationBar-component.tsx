import React, { useState } from 'react';
import { Divider,Typography, Button, Grid } from '@material-ui/core';

import NotificationBar from '../../components/notification-bar';
import { NotificationBarProps } from '../../components/notification-bar';
import { styleTyping } from './Utils';

const styles: styleTyping = {
  root: {
    margin: '0 auto',
    width: '40%',
  },
  heading: {
    padding: '0.5rem'
  },
  button: {
    margin: '1rem auto'
  }
}

export default function NotificationBarStory(props: NotificationBarProps) {

  const {message, type} = props;
  const [showNotifications, setShowNotifications] = useState(false);
  const [disableButton, setDisableButton] = useState(false);

  const handleShowNotifications = () => {
    setShowNotifications(true);
    setDisableButton(true);

    setTimeout(() => {
      setShowNotifications(false);
      setDisableButton(false);
    }, 3100);
  }

  return (
    <div style={styles.root}>
      <div style={styles.heading}>
        <Typography variant="body1">
          The notification bar contains 3 props: <b>message</b>, <b>type </b>
          and an optional <b>time</b>. The notification bar has a display duration 
          of 3 seconds before it auto hides. It will display the message and the background 
          color will be determined by the type that is passed in. To see the bar, please press
          the button below.
        </Typography>
        <Grid container direction="row" spacing={1} alignItems="center" justify="center">
          <Button onClick={handleShowNotifications} 
            disabled={disableButton} 
            variant="contained" 
            color="primary"
            style={styles.button}>
              See Notification
          </Button>
        </Grid>
      </div>
      <Divider />
      {
        showNotifications ? <NotificationBar message={message} type={type} /> : null
      }
    </div>
  )
}
