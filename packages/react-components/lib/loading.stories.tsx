import { Button, Typography } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { Loading, LoadingProps } from './loading';

export default {
  title: 'Loading',
  component: Loading,
  argTypes: {
    loading: {
      defaultValue: false,
    },
  },
} satisfies Meta;

export const LoadingButton: StoryFn<LoadingProps> = (args) => {
  return (
    <>
      <Button variant="contained" disabled={args.loading}>
        <Loading {...args} size="1.5em" hideChildren>
          OK
        </Loading>
      </Button>
      <Typography style={{ marginTop: 8 }}>
        Use the storybook controls to change loading state
      </Typography>
    </>
  );
};
