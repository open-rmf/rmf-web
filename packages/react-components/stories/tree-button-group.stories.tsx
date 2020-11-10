import { Story, Meta } from '@storybook/react';
import React from 'react';
import { TreeButtonGroup } from '../lib';

export default {
  title: 'Tree Button Group',
  component: TreeButtonGroup,
} as Meta;

export const SimpleTreeButtonGroup = (): React.ReactElement => {
  return <TreeButtonGroup />;
};
