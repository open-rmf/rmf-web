import React from 'react';
import { Story, Meta } from '@storybook/react';
import NavigationBar from '../lib/navigation-bar';

export default {
  title: 'Navigation Bar',
  component: NavigationBar,
} as Meta;

export const SimpleNavigationBar: Story = (args) => {
  const tabNames = ['Robots', 'Tasks', 'History'];
  return <NavigationBar tabNames={tabNames} {...args} />;
};
