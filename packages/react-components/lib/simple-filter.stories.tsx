import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { OnFilterChangeEvent, SimpleFilter } from './simple-filter';

export default {
  title: 'Simple Filter',
  component: SimpleFilter,
} satisfies Meta;

function SimpleFilterHandler(): JSX.Element {
  const [filter, setFilter] = React.useState('');

  const onChange = (e: React.ChangeEvent<OnFilterChangeEvent>) => {
    setFilter(e.target.value);
  };

  return <SimpleFilter value={filter} onChange={onChange} />;
}

export const SimpleFilterStory: StoryFn = (args) => {
  return <SimpleFilterHandler {...args} />;
};
