import React from 'react';
import { AlertProps } from '@material-ui/lab/Alert';
import { Divider,Typography } from '@material-ui/core';

import NotificationBar from '../../components/notification-bar';

export interface notificationBarProps {
    message: string | undefined | null;
    type: AlertProps['severity'];
    time?: string;
}

export interface styleTyping {
    root: React.CSSProperties;
    heading: React.CSSProperties;
}


const styles: styleTyping = {
    root: {
        margin: '0 auto',
        width: '40%',
    },
    heading: {
        padding: '0.5rem'
    }
}

export default function NotificationBarStory(props: notificationBarProps) {

    const {message, type} = props;

    return (
        <div style={styles.root}>
            <div style={styles.heading}>
                <Typography variant="body1">
                    The notification bar contains 3 props: <b>message</b>, <b>type </b>
                    and an optional <b>time</b>. The notification bar has a display duration 
                    of 3 seconds before it auto hides. It will display the message and the background 
                    color will be determined by the type that is passed in. To see the bar, you have to 
                    refresh the browser.

                </Typography>
            </div>
            <Divider />
            <NotificationBar message={message} type={type} />
        </div>
    )
}
