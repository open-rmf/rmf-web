import React from 'react';
import { Divider,Typography } from '@material-ui/core';

import { styleTyping, sampleStyleTyping } from './BaseComponents/Utils';

export default {
  title: 'Color Palette',
};

const styles: styleTyping = {
  root: {
    margin: '0 auto',
    width: '40%',
  },
  heading: {
    padding: '0.5rem'
  },
  example: {
    display: 'flex',
    justifyContent: 'space-between',
    margin: '1rem 0'
  },
  aTag: {
    textDecoration: 'none',
    color: 'rgb(20, 116, 243)'
  },
};

const sampleStyles: sampleStyleTyping = {
  colorSample: {
    unknown: {
      backgroundColor: '#cccccc',
      padding: '0.5rem',
    },
  }
}

export const colors = () => (
  <div style={styles.root}>
    <div style={styles.heading}>
      <Typography variant="body1">
        This section contains the palette of colors we use in our app 
        for consistent design across components and state representation.
      </Typography>
    </div>
    <Divider />
    <div style={styles.example}>
      <Typography variant="body1">Unknown</Typography>
      <div>
        <Typography variant="body1">Color code: #cccccc</Typography>
        <div style={sampleStyles.colorSample.unknown} />
      </div>
    </div>
    <Divider />
    <div style={styles.example}>
      <Typography variant="body1">Material UI colors</Typography>
      <Typography variant="body1">
        Refer to <a style={styles.aTag} href="https://material-ui.com/customization/palette/">here</a> for other colors we use.
      </Typography>
    </div>
  </div>
)