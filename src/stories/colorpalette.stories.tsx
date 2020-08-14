import React from 'react';
import { Divider, Typography } from '@material-ui/core';

import { StyleTyping, defaultStyles } from './BaseComponents/Utils';
import Color from './BaseComponents/color';

export default {
  title: 'Color Palette',
};

const styles: StyleTyping = {
  ...defaultStyles,
  example: {
    display: 'flex',
    justifyContent: 'space-between',
    margin: '1rem 0',
  },
};

export const colors = () => (
  <div style={styles.root}>
    <div style={styles.heading}>
      <Typography variant="body1">
        This section contains the palette of colors we use in our app for consistent design across
        components and state representation. Refer to{' '}
        <a style={styles.aTag} href="https://material-ui.com/customization/palette/">
          here
        </a>{' '}
        for other colors we use from <b>Material UI</b>.
      </Typography>
    </div>
    <Divider />
    <Color />
  </div>
);
