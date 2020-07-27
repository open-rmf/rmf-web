import React from 'react';
import { Divider,Typography } from '@material-ui/core';

import ButtonColors from './BaseComponents/button-colors';

export default {
    title: 'Design Decisions',
 };

export interface styleTyping {
    root: React.CSSProperties;
    example: React.CSSProperties;
    ellipsis: React.CSSProperties;
    explanation: React.CSSProperties;
    panelButton: React.CSSProperties;
}

const styles: styleTyping = {
    root: {
        margin: '0 auto',
        width: '40%',
    },
    example: {
        display: 'flex',
        justifyContent: 'space-between',
        margin: '1rem 0'
    },
    ellipsis: {
        border: '1px solid black',
        padding: '0.5rem',
        overflow: "hidden",
        textOverflow: "ellipsis",
        whiteSpace: "nowrap",
        maxWidth: "10rem",
    },
    explanation: {
        margin: '1rem 0',
    },
    panelButton: {
        border: '2px solid #CCCCCC',
        padding: 5,
        width: '4rem',
        textAlign: 'center',
        borderRadius: '4px'
    }
}

const doorOpen: React.CSSProperties = {
    ... styles.panelButton,
    border: '2px solid #4caf50'
}

const doorMoving: React.CSSProperties = {
    ... styles.panelButton,
    border: '2px solid #ff9800'
}

const doorClosed: React.CSSProperties = {
    ... styles.panelButton,
    border: '2px solid #f44336'
}

const dispenserOnline: React.CSSProperties = {
    ... styles.panelButton,
    border: '2px solid #4caf50'
}

const dispenserIdle: React.CSSProperties = {
    ... styles.panelButton,
    border: '2px solid #ff9800'
}

const dispenserOffline: React.CSSProperties = {
    ... styles.panelButton,
    border: '2px solid #f44336'
}

const liftStop: React.CSSProperties = {
    ... styles.panelButton,
    border: '2px solid #2196f3'
}
const liftMoving: React.CSSProperties = {
    ... styles.panelButton,
    border: '2px solid #ff9800'
}

const robotStates: React.CSSProperties = {
    padding: 5,
    minWidth: '4rem',
    textAlign: 'center',
    borderRadius: '4px',
    border: '2px solid #2196f3'
}
  
export const HandleLongName = () => (
    <div style={styles.root}>
        <Typography style={styles.explanation} variant="body1">
            Since the names of the items have the potential to be longer than
            the container they are in, we truncate it with an ellipsis if it exceeds
            and also included a <b>Name</b> field in the detail panel.
        </Typography>

        <Divider />

        <div style={styles.example}>
            <Typography variant="body1">An example:</Typography>
            <Typography style={styles.ellipsis} variant="body1">A very very long name</Typography>
        </div>
    </div>
)

export const HandleUnknown = () => (
    <div style={styles.root}>
        <Typography style={styles.explanation} variant="body1">
            Sometimes, device states might be returned as <b>Unknown</b> for various reasons. As <b>Unknown </b> 
            is too long for the width of the Panel button, we display it as <b>N/A </b> 
            in the panel button with a greyed out border.
        </Typography>

        <Divider />

        <div style={styles.example}>
            <Typography variant="body1">An example:</Typography>
            <Typography style={styles.panelButton} data-role="state" variant="button">N/A</Typography>
        </div>
    </div>
)

export const DoorButtonColors = () => (
    <div style={styles.root}>
        <div style={styles.example}>
            <Typography variant="h6">Door State</Typography>
            <Typography variant="h6">Button color and representation</Typography>
        </div>
        <Divider />
        <ButtonColors state={'OPEN'} style={doorOpen} />
        <Divider />
        <ButtonColors state={'MOVING'} style={doorMoving} />
        <Divider />
        <ButtonColors state={'CLOSED'} style={doorClosed} />
    </div>
)

export const DispenserButtonColors = () => (
    <div style={styles.root}>
        <div style={styles.example}>
            <Typography variant="h6">Dispenser State</Typography>
            <Typography variant="h6">Button color and representation</Typography>
        </div>
        <Divider />
        <ButtonColors state={'ONLINE'} style={dispenserOnline} />
        <Divider />
        <ButtonColors state={'IDLE'} style={dispenserIdle} />
        <Divider />
        <ButtonColors state={'OFFLINE'} style={dispenserOffline} />
    </div>
)

export const LiftButtonColors = () => (
    <div style={styles.root}>
        <div style={styles.example}>
            <Typography variant="h6">Lift State</Typography>
            <Typography variant="h6">Button color and representation</Typography>
        </div>
        <Divider />
        <ButtonColors state={'STOP'} style={liftStop} />
        <Divider />
        <ButtonColors state={'MOVING'} style={liftMoving} />
    </div>
)

export const RobotButtonColors = () => (
    <div style={styles.root}>
        <div style={styles.example}>
            <Typography variant="h6">Robot State</Typography>
            <Typography variant="h6">Button color and representation</Typography>
        </div>
        <Divider />
        <ButtonColors state={'CHARGING'} style={robotStates} />
        <Divider />
        <ButtonColors state={'DOCKING'} style={robotStates} />
        <Divider />
        <ButtonColors state={'EMERGENCY'} style={robotStates} />
        <Divider />
        <ButtonColors state={'GOING HOME'} style={robotStates} />
        <Divider />
        <ButtonColors state={'IDLE'} style={robotStates} />
        <Divider />
        <ButtonColors state={'MOVING'} style={robotStates} />
        <Divider />
        <ButtonColors state={'PAUSED'} style={robotStates} />
        <Divider />
        <ButtonColors state={'WAITING'} style={robotStates} />
    </div>
)
