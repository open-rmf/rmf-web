import React from 'react';
import { Divider, Typography } from '@material-ui/core';

import { StyleTyping, defaultStyles } from './BaseComponents/Utils';

export default {
  title: 'Design Decisions',
};

const styles: StyleTyping = {
  ...defaultStyles,
  example: {
    display: 'flex',
    justifyContent: 'space-between',
    margin: '1rem 0',
  },
  ellipsis: {
    border: '1px solid black',
    padding: '0.5rem',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
    whiteSpace: 'nowrap',
    maxWidth: '10rem',
  },
  explanation: {
    margin: '1rem 0',
  },
  panelButton: {
    border: '2px solid #CCCCCC',
    padding: 5,
    width: '4rem',
    textAlign: 'center',
    borderRadius: '4px',
  },
};

export const HandleLongName = () => (
  <div style={styles.root}>
    <Typography style={styles.explanation} variant="body1">
      Since the names of the items have the potential to be longer than the container they are in,
      we truncate it with an ellipsis if it exceeds and also included a <b>Name</b> field in the
      detail panel.
    </Typography>

    <Divider />

    <div style={styles.example}>
      <Typography variant="body1">An example:</Typography>
      <Typography style={styles.ellipsis} variant="body1">
        A very very long name
      </Typography>
    </div>
  </div>
);

export const HandleUnknown = () => (
  <div style={styles.root}>
    <Typography style={styles.explanation} variant="body1">
      Sometimes, device states might be returned as <b>Unknown</b> for various reasons. As{' '}
      <b>Unknown </b>
      is too long for the width of the Panel button, we display it as <b>N/A </b>
      in the panel button with a greyed out border.
    </Typography>

    <Divider />

    <div style={styles.example}>
      <Typography variant="body1">An example:</Typography>
      <Typography style={styles.panelButton} data-role="state" variant="button">
        N/A
      </Typography>
    </div>
  </div>
);
