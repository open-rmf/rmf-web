import AndroidIcon from '@mui/icons-material/Android';
import ArrowDropUpIcon from '@mui/icons-material/ArrowDropUp';
import SearchIcon from '@mui/icons-material/Search';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { ExpandableMultilevelMenuProps, MultiLevelMenu } from './multi-level-menu';

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
  <MultiLevelMenu menuStructure={menuStructure} {...args} />
);
