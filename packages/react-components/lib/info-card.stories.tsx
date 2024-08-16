import { styled } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';

import { SimpleInfo } from '../lib';

const classes = {
  background: 'info-card-root',
};

const InfoCardRoot = styled('div')(({ theme }) => ({
  [`&.${classes.background}`]: {
    backgroundColor: theme.palette.background.paper,
    color: theme.palette.text.primary,
  },
}));

export default {
  title: 'Simple Info',
  component: SimpleInfo,
  argTypes: {
    stringDisplayName: {
      name: 'String Display Name',
    },
    stringValue: {
      name: 'String Value',
    },
    numberDisplayName: {
      name: 'Number Display Name',
    },
    numberValue: {
      name: 'Number Value',
    },
  },
} satisfies Meta;

type Story = StoryObj<typeof SimpleInfo>;

export const StringData: Story = {
  render: (args) => (
    <InfoCardRoot className={classes.background}>
      <SimpleInfo {...args} infoData={[{ name: 'string', value: 'value' }]} />
    </InfoCardRoot>
  ),
};

export const ArrayData: Story = {
  render: (args) => (
    <InfoCardRoot className={classes.background}>
      <SimpleInfo {...args} infoData={[{ name: 'strings', value: ['hello', 'world'] }]} />
    </InfoCardRoot>
  ),
};
