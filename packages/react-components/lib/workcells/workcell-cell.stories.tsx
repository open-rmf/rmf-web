import { Meta, Story } from '@storybook/react';
import React from 'react';
import { WorkcellCell, WorkcellCellProps } from './workcell-cell';

export default {
  title: 'Workcell Cell',
  component: WorkcellCell,
} as Meta;

export const Default: Story<WorkcellCellProps> = (args) => {
  return <WorkcellCell {...args} />;
};

Default.storyName = 'Workcell Cell';

Default.args = {
  guid: 'test_workcell',
  requestGuidQueue: [],
  secondsRemaining: 0,
};
