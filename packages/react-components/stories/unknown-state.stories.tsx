import { Story } from '@storybook/react';
import React from 'react';
import { Divider, Typography } from '@material-ui/core';
import { DispenserAccordion } from '../lib';

export default {
  title: 'Unknown Item States representation',
};

const statelessDispenserGuid = 'stateless dispenser';
const styles: Record<string, React.CSSProperties> = {
  root: {
    margin: '0 auto',
  },
  spacing: {
    margin: '1rem 0',
  },
};

export const itemsWithUnknownState: Story = (args) => (
  <div>
    <Typography variant="body1">
      There are situations where the state of a component (for example, dispenser or robots) is not
      known due to them being disconnected from RMF or other reasons, resulting in them not being
      displayed in the omnipanel.
      <br />
      <b>ItemUnknownState</b> component provides a visual to notify these end users that these
      components are not registered. Below is an example of it in a dispenser accordian.
    </Typography>
    <Divider style={{ margin: '1rem 0' }} />
    <div style={{ ...styles.spacing, width: 400 }}>
      <DispenserAccordion dispenser={statelessDispenserGuid} dispenserState={null} {...args} />
    </div>
  </div>
);
