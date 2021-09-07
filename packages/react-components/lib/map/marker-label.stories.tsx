import { Meta, Story } from '@storybook/react';
import React from 'react';
import { MarkerContainer, NameLabel, NameLabelProps } from './marker-label';

export default {
  title: 'Map/Marker Labels',
  component: MarkerContainer,
} as Meta;

export const NameLabelStory: Story<NameLabelProps> = (args) => {
  return (
    <svg viewBox="-4 -2 8 4" width={800} height={400} style={{ border: '1px black solid' }}>
      <NameLabel x={0} y={0} {...args} />
    </svg>
  );
};
NameLabelStory.storyName = 'Name Label';
NameLabelStory.args = {
  text: 'test name',
};
