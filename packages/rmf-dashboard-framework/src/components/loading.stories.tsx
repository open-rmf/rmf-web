import { Button, Typography } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';

import { Loading } from './loading';

export default {
  title: 'Loading',
  component: Loading,
  argTypes: {
    loading: {
      defaultValue: false,
    },
  },
} satisfies Meta;

type Story = StoryObj<typeof Loading>;

export const LoadingButton: Story = {
  render: (args) => (
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
  ),
};
