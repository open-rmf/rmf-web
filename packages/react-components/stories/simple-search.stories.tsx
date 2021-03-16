import { Meta, Story } from '@storybook/react';
import React from 'react';
import { SimpleSearch, OnChangeEvent } from '../lib/index';

export default {
  title: 'Simple Search',
  component: SimpleSearch,
} as Meta;

function SimpleSearchHandler(): JSX.Element {
  const [search, setSearch] = React.useState('');

  const onChange = (e: React.ChangeEvent<OnChangeEvent>) => {
    setSearch(e.target.value);
  };

  return <SimpleSearch value={search} onChange={onChange} />;
}

export const SimpleSearchStory: Story = (args) => {
  return <SimpleSearchHandler {...args} />;
};
