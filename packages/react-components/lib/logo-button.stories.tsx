import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { LogoButton } from './logo-button';

export default {
  title: 'Logo Button',
  component: LogoButton,
} satisfies Meta;

export const Default: StoryFn = () => {
  return <LogoButton src="/assets/roshealth-logo-white.png" />;
};

Default.storyName = 'Logo Button';
