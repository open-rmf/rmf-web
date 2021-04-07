import React from 'react';
import { Story, Meta } from '@storybook/react';
import TitleBar from '../lib/title-bar';

export default {
  title: 'Title Bar',
  component: TitleBar,
} as Meta;

export const SimpleTooltip: Story = (args) => {
  return <TitleBar logoPath={'/resources/roshealth-logo-white.png'} {...args} />;
};
