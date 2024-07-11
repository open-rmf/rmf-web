import { styled } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { ErrorOverlay } from './error-overlay';
import { SimpleInfo, SimpleInfoProps } from './simple-info';

export default {
  title: 'Error Overlay',
  component: ErrorOverlay,
} satisfies Meta;

const classes = {
  container: 'simple-info-testcomponent',
};
const SimpleInfo_ = styled((props: SimpleInfoProps) => <SimpleInfo {...props} />)(() => ({
  [`& .${classes.container}`]: {
    display: 'table',
    borderCollapse: 'collapse',
    width: '100%',
    overflowX: 'auto',
    userSelect: 'none',
  },
}));

function TestComponent() {
  const data = [
    { name: 'String', value: 'This is a string' },
    { name: 'Number', value: 3 },
    {
      name: 'Array',
      value: ['one', 'two', 'three'],
    },
  ];
  return <SimpleInfo_ infoData={data} overrideStyle={classes} />;
}

export const ErrorOverlayPanel: StoryFn = (args) => {
  return (
    <>
      <ErrorOverlay errorMsg={'This is an error message'} {...args}>
        <TestComponent />
      </ErrorOverlay>
    </>
  );
};
