import React from 'react';
import { Divider, Typography } from '@mui/material';

import { colorPaletteUtil, defaultStyles, StyleTyping } from './utils';

const styles: StyleTyping = {
  ...defaultStyles,
  example: {
    display: 'flex',
    justifyContent: 'space-between',
    margin: '1rem 0',
    padding: '0 0.5rem',
  },
};

export default function Colors() {
  return (
    <React.Fragment>
      {Object.keys(colorPaletteUtil).map((color, index) => {
        return (
          <React.Fragment>
            <div style={styles.example} key={index}>
              <Typography variant="body1">Unknown</Typography>
              <div>
                <Typography variant="body1">{`Color code: ${colorPaletteUtil[color].backgroundColor}`}</Typography>
                <div style={{ ...colorPaletteUtil[color], padding: '0.5rem' }} />
              </div>
            </div>
            <Divider />
          </React.Fragment>
        );
      })}
    </React.Fragment>
  );
}
