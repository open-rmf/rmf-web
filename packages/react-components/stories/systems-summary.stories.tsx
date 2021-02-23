import React from 'react';
import {
  SystemSummaryItemState,
  SystemSummaryItemStateProps,
  SystemSummaryAlertProps,
  SystemSummaryAlert,
  SystemSummaryTaskState,
  SystemSummaryTaskStateProps,
  SystemSummaryBanner,
  SystemSummaryBannerProps,
  SystemSummarySpoiltItems,
  SystemSummarySpoiltItemsProps,
  Severity,
  RobotSummaryState,
  RobotSummaryStateProps,
} from '../lib';
import { Meta, Story } from '@storybook/react';

export default {
  title: 'Systems summary',
  component: SystemSummaryItemState,
} as Meta;

const itemStateDataDoor: SystemSummaryItemStateProps = {
  itemSummary: {
    operational: 0,
    spoiltItem: [],
  },
  onClick: () => {
    /**filler */
  },
  item: 'Doors',
};

const itemStateDataRobot: RobotSummaryStateProps = {
  itemSummary: {
    operational: 0,
    idle: 0,
    charging: 0,
    spoiltRobots: [],
  },
  onClick: () => {
    /**filler */
  },
  item: 'Robots',
};

const systemSummaryAlert: SystemSummaryAlertProps = {
  notifications: [
    { id: 1, severity: Severity.High, time: 'January 29th 2021, 8:20:50', error: 'hello world' },
  ],
  onNotificationsDismiss: () => {
    /**filler */
  },
};

const systemSummaryTaskStateData: SystemSummaryTaskStateProps = {
  tasks: [
    {
      task_id: 'abc',
      state: 0,
      start_time: { sec: 0, nanosec: 0 },
      end_time: { sec: 0, nanosec: 0 },
      status: 'status',
      submission_time: { sec: 0, nanosec: 0 },
    },
    {
      task_id: 'abc',
      state: 1,
      start_time: { sec: 0, nanosec: 0 },
      end_time: { sec: 0, nanosec: 0 },
      status: 'status',
      submission_time: { sec: 0, nanosec: 0 },
    },
    {
      task_id: 'abc',
      state: 2,
      start_time: { sec: 0, nanosec: 0 },
      end_time: { sec: 0, nanosec: 0 },
      status: 'status',
      submission_time: { sec: 0, nanosec: 0 },
    },
    {
      task_id: 'abc',
      state: 3,
      start_time: { sec: 0, nanosec: 0 },
      end_time: { sec: 0, nanosec: 0 },
      status: 'status',
      submission_time: { sec: 0, nanosec: 0 },
    },
  ],
  onClick: () => {
    /**filler */
  },
};

const systemSummaryBannerData: SystemSummaryBannerProps = {
  isError: false,
};

const spoiltEquipment: SystemSummarySpoiltItemsProps = {
  doors: [
    {
      door: {
        name: 'hardware_door',
        v1_x: 4.9,
        v1_y: -4,
        v2_x: 4.4,
        v2_y: -5,
        door_type: 1,
        motion_range: 1.571,
        motion_direction: -1,
      },
      name: 'hardware_door',
      state: 'unknown',
    },
  ],
  lifts: [],
  dispensers: [],
  robots: [],
};

const onSpoiltItemClick = () => {
  // filler
};

export const SystemSummaryItemStateStory: Story = (args) => (
  <React.Fragment>
    <SystemSummaryItemState
      item={itemStateDataDoor.item}
      itemSummary={itemStateDataDoor.itemSummary}
      onClick={itemStateDataDoor.onClick}
      {...args}
    />
    <RobotSummaryState
      item={itemStateDataRobot.item}
      itemSummary={itemStateDataRobot.itemSummary}
      onClick={itemStateDataRobot.onClick}
      {...args}
    />
  </React.Fragment>
);

export const SystemSummaryItemAlertStory: Story = (args) => (
  <React.Fragment>
    <SystemSummaryAlert
      notifications={systemSummaryAlert.notifications}
      onNotificationsDismiss={systemSummaryAlert.onNotificationsDismiss}
      {...args}
    />
    <SystemSummaryAlert
      notifications={[]}
      onNotificationsDismiss={systemSummaryAlert.onNotificationsDismiss}
      {...args}
    />
  </React.Fragment>
);

export const SystemSummaryTaskStateStory: Story = (args) => (
  <SystemSummaryTaskState
    tasks={systemSummaryTaskStateData.tasks}
    onClick={systemSummaryTaskStateData.onClick}
    {...args}
  />
);

export const SystemSummaryBannerStory: Story = (args) => (
  <React.Fragment>
    <SystemSummaryBanner
      isError={systemSummaryBannerData.isError}
      imageSrc={'link-to-avatar'}
      {...args}
    />
    <div style={{ margin: '1rem 0' }}></div>
    <SystemSummaryBanner isError={!systemSummaryBannerData.isError} {...args} />
  </React.Fragment>
);

export const SystemSummarySpoiltItemStory: Story = (args) => (
  <SystemSummarySpoiltItems
    doors={spoiltEquipment.doors}
    lifts={spoiltEquipment.lifts}
    dispensers={spoiltEquipment.dispensers}
    robots={spoiltEquipment.robots}
    onClickSpoiltDoor={onSpoiltItemClick}
    {...args}
  />
);
