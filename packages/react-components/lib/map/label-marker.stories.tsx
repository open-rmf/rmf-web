import { Meta, Story } from '@storybook/react';
import React from 'react';
import { NameLabel, NameLabelProps } from './label-marker';

export default {
  title: 'Map/Label Marker',
} as Meta;

export const NameLabelStory: Story<NameLabelProps> = (args) => {
  return (
    <svg viewBox="-4 -2 8 4" width={800} height={400} style={{ border: '1px black solid' }}>
      <NameLabel {...args} />
    </svg>
  );
};
NameLabelStory.storyName = 'Name Label';
NameLabelStory.args = {
  text: 'test name',
  anchorX: 0,
  anchorY: 0,
  arrowLength: 0.5,
  theta: -45,
  radius: 0.05,
  fontSize: 0.3,
  stroke: 'black',
  strokeWidth: 0.03,
};
