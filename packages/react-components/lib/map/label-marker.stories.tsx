import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { NameLabel, NameLabelProps } from './label-marker';

export default {
  title: 'Map/Label Marker',
} satisfies Meta;

export const NameLabelStory: StoryFn<NameLabelProps> = (args) => {
  return (
    <svg width={800} height={400} style={{ border: '1px black solid' }}>
      <NameLabel {...args} strokeWidth={1} style={{ fontSize: '1em' }} />
    </svg>
  );
};
NameLabelStory.storyName = 'Name Label';
NameLabelStory.argTypes = {
  contentBorderRadius: {
    control: { type: 'number' },
  },
  angle: {
    control: { type: 'number' },
  },
  arrowLength: {
    control: { type: 'number' },
  },
};
NameLabelStory.args = {
  text: 'test name',
  sourceX: 400,
  sourceY: 200,
  sourceRadius: 0,
  angle: undefined,
  contentBorderRadius: undefined,
  arrowLength: undefined,
};
