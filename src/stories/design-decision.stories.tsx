import React from 'react';
import { Divider, Typography } from '@material-ui/core';

import { StyleTyping, defaultStyles, dispenserStates } from './baseComponents/utils';
import DispenserButton from './baseComponents/dispenser-panel';

export default {
  title: 'Design Decisions',
};

const styles: StyleTyping = {
  ...defaultStyles,
  spacing: {
    margin: '1rem 0',
  },
};

const longName = { second_dispenser: dispenserStates['second_dispenser'] };
const unknownState = { fourth_dispenser: dispenserStates['fourth_dispenser'] };

export const handleLongName = () => (
  <div style={styles.root}>
    <Typography style={styles.spacing} variant="body1">
      Since the names of the items have the potential to be longer than the container they are in,
      we truncate it with an ellipsis if it exceeds and also included a <b>Name</b> field in the
      detail panel. Shown below is an example on the dispenser panel.
    </Typography>
    <Divider />
    <div style={styles.spacing}>
      <DispenserButton dispenserStates={longName} />
    </div>
  </div>
);

export const handleUnknown = () => (
  <div style={styles.root}>
    <Typography style={styles.spacing} variant="body1">
      Sometimes, device states might be returned as <b>Unknown</b> for various reasons. As{' '}
      <b>Unknown </b>
      is too long for the width of the Panel button, we display it as <b>N/A </b>
      in the panel button with a greyed out border. Shown below is an example on the dispenser
      panel.
    </Typography>
    <Divider />
    <div style={styles.spacing}>
      <DispenserButton dispenserStates={unknownState} />
    </div>
  </div>
);
