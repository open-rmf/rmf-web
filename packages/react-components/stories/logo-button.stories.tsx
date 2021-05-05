import React from 'react';
import { Story, Meta } from '@storybook/react';
import { LogoButton } from '../lib/logo-button';
import Typography from '@material-ui/core/Typography';

export default {
  title: 'Logo Button',
  component: LogoButton,
} as Meta;

export const ClickEnabledLogoButton: Story = () => {
  return (
    <>
      <Typography variant="h6">Click the button below</Typography>
      <LogoButton
        logoPath="/resources/roshealth-logo-white.png"
        onClick={() => alert('hello world')}
      />
    </>
  );
};

export const DisabledLogoButton: Story = () => {
  return (
    <>
      <Typography variant="h6">This button is disabled</Typography>
      <LogoButton logoPath="/resources/roshealth-logo-white.png" />
    </>
  );
};
