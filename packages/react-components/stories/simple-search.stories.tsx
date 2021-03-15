import { Meta, Story } from '@storybook/react';
import React from 'react';
import { SimpleSearch, OnChangeEvent } from '../lib/index';

export default {
  title: 'Simple Search',
  component: SimpleSearch,
} as Meta;

interface SimpleSearchHandlerProps {
  disabled: boolean;
}

function SimpleSearchHandler(props: SimpleSearchHandlerProps): JSX.Element {
  const { disabled } = props;
  const [search, setSearch] = React.useState('');

  const onChange = (e: React.ChangeEvent<OnChangeEvent>) => {
    setSearch(e.target.value);
  };

  const deleteSearchTerm = () => {
    setSearch('');
  };

  return (
    <SimpleSearch
      disabled={disabled}
      value={search}
      onChange={onChange}
      onClick={deleteSearchTerm}
    />
  );
}

export const SimpleSearchStory: Story = (args) => {
  return (
    <React.Fragment>
      <SimpleSearchHandler disabled={false} {...args} />
      <SimpleSearchHandler disabled={true} {...args} />
    </React.Fragment>
  );
};
