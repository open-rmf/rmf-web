import { Meta, Story } from '@storybook/react';
import React from 'react';
import { LogoButton } from './logo-button';

export default {
  title: 'Logo Button',
  component: LogoButton,
} as Meta;

export const Default: Story = () => {
  return <LogoButton src="/assets/roshealth-logo-white.png" />;
};

Default.storyName = 'Logo Button';
