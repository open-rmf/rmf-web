import { Meta, Story } from '@storybook/react';
import React from 'react';
import { NameLabel, NameLabelProps } from './marker-label';

export default {
  title: 'Map/Marker Labels',
} as Meta;

export const NameLabelStory: Story<NameLabelProps> = (args) => {
  return (
    <svg viewBox="-4 -2 8 4" width={800} height={400} style={{ border: '1px black solid' }}>
      <NameLabel {...args} anchorX={0} anchorY={0} transform="scale(8)" />
    </svg>
  );
};
NameLabelStory.storyName = 'Name Label';
NameLabelStory.args = {
  text: 'test name',
};
