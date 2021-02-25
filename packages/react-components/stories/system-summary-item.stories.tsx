import React from 'react';
import { SystemSummaryItemState, ItemSummary, RobotSummary, RobotSummaryState } from '../lib';
import { Meta, Story } from '@storybook/react';

export default {
  title: 'Systems summary item',
  component: SystemSummaryItemState,
} as Meta;

const itemSummary: ItemSummary = {
  operational: 0,
  spoiltItem: [],
};

const robotSummary: RobotSummary = {
  operational: 0,
  idle: 0,
  charging: 0,
  spoiltRobots: [],
};

export const SystemSummaryItemStateStory: Story = (args) => (
  <React.Fragment>
    <SystemSummaryItemState
      item={'Door'}
      itemSummary={itemSummary}
      onClick={() => {
        /**filler */
      }}
      {...args}
    />
    <RobotSummaryState
      item={'Robots'}
      itemSummary={robotSummary}
      onClick={() => {
        /**filler */
      }}
      {...args}
    />
  </React.Fragment>
);
