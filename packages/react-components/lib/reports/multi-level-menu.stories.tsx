import AndroidIcon from '@material-ui/icons/Android';
import ArrowDropUpIcon from '@material-ui/icons/ArrowDropUp';
import { styled } from '@material-ui/core';
import SearchIcon from '@material-ui/icons/Search';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { ExpandableMultilevelMenuProps, MultiLevelMenu } from './multi-level-menu';

const classes = {
  container: 'mlm-story-container',
};

const MlmStory = styled('div')(({ theme }) => ({
  [`&.${classes.container}`]: {
    backgroundColor: theme.palette.background.paper,
  },
}));

export default {
  title: 'Multi level menu',
} as Meta;

const menuStructure = [
  {
    icon: <SearchIcon />,
    title: 'All logs',
    items: [],
  },
  {
    icon: <AndroidIcon />,
    title: 'Robots',
    items: [
      {
        title: 'Robot states',
        items: [],
      },
    ],
  },
  {
    icon: <ArrowDropUpIcon />,
    title: 'Test',
    items: [
      {
        title: 'Test-child',
        items: [],
      },
    ],
  },
] as ExpandableMultilevelMenuProps[];

export const MultiLevelMenuStory: Story = (args) => (
  <MlmStory className={classes.container}>
    <MultiLevelMenu menuStructure={menuStructure} {...args} />
  </MlmStory>
);
