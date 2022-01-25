import { Divider, Typography } from '@mui/material';
import { Story } from '@storybook/react';
import React from 'react';
import { StatusLabel } from './status-label';

export default {
  title: 'Design Decisions',
};

const styles: Record<string, React.CSSProperties> = {
  spacing: {
    margin: '1rem 0',
  },
};

export const handleUnknown: Story = () => (
  <div>
    <Typography style={styles.spacing} variant="body1">
      Sometimes, device states might be returned as <b>Unknown</b> for various reasons. As{' '}
      <b>Unknown </b>
      is too long for the width of the status label, we display it as <b>N/A </b>
      with a greyed out border instead.
    </Typography>
    <Divider style={{ margin: '1rem 0' }} />
    <StatusLabel variant="unknown" />
  </div>
);
