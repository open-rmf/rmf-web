import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { SimpleInfo } from '../lib';
import { styled } from '@mui/material';

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

export const SimpleData: StoryFn = ({
  stringDisplayName,
  stringValue,
  numberDisplayName,
  numberValue,
  ...args
}) => {
  return (
    <InfoCardRoot className={classes.background}>
      <SimpleInfo
        infoData={[
          { name: stringDisplayName, value: stringValue },
          { name: numberDisplayName, value: numberValue },
        ]}
        {...args}
      />
    </InfoCardRoot>
  );
};
SimpleData.args = {
  stringDisplayName: 'string',
  stringValue: 'value',
  numberDisplayName: 'number',
  numberValue: 0,
};

export const Array: StoryFn = (args) => {
  return (
    <InfoCardRoot className={classes.background}>
      <SimpleInfo infoData={[{ name: 'strings', value: ['hello', 'world'] }]} {...args} />
    </InfoCardRoot>
  );
};
