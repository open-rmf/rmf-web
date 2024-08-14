import { Meta, StoryObj } from '@storybook/react';

import { LogoButton } from './logo-button';

export default {
  title: 'Logo Button',
  component: LogoButton,
} satisfies Meta;

type Story = StoryObj<typeof LogoButton>;

export const Default: Story = {
  storyName: 'Logo Button',
  render: () => <LogoButton src="/assets/roshealth-logo-white.png" />,
};
