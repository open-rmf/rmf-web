import React from 'react';
import { Button, Typography, Divider } from '@material-ui/core';

import { StyleTyping, defaultStyles } from './utils';

const styles: StyleTyping = {
  ...defaultStyles,
  button: {
    width: '100%',
  },
  buttonWrapper: {
    padding: '1rem 0.5rem',
  },
};

export default function FormButtonComponent() {
  return (
    <div style={styles.root}>
      <div style={styles.heading}>
        <Typography variant="body1">
          Below is an example of the buttons that we are using for our forms. Refer to{' '}
          <a style={styles.aTag} href="https://material-ui.com/components/buttons/">
            here
          </a>{' '}
          for more examples and information of the API.
        </Typography>
      </div>
      <Divider />
      <div style={styles.buttonWrapper}>
        <Button style={styles.button} variant="contained" color="primary">
          {'Request'}
        </Button>
      </div>
    </div>
  );
}
