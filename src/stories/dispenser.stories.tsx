import React from 'react';
import { Typography } from '@material-ui/core';

import DispenserButton from './BaseComponents/dispenser-panel';
import { dispenserStates, defaultStyles, StyleTyping } from './BaseComponents/Utils';

export default {
  title: 'Dispenser',
};

const styles: StyleTyping = {
  ...defaultStyles,
  example: {
    display: 'flex',
    justifyContent: 'space-between',
    margin: '1rem 0',
  },
};

export const DispenserPanel = () => (
  <div style={styles.root}>
    <div style={styles.example}>
      <Typography variant="h6">Dispenser State</Typography>
      <Typography variant="h6">Button color and representation</Typography>
    </div>
    <DispenserButton dispenserStates={dispenserStates} />
  </div>
);
