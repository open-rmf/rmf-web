import React from 'react';
import {
  MainMenuItemState,
  MainMenuItemStateProps,
  MainMenuAlertProps,
  MainMenuAlert,
  MainMenuTaskState,
  MainMenuTaskStateProps,
  MainMenuBanner,
  MainMenuBannerProps,
} from '../lib';
import { Meta, Story } from '@storybook/react';

export default {
  title: 'Main Menu Item State',
  component: MainMenuItemState,
} as Meta;

const itemStateDataDoor: MainMenuItemStateProps = {
  itemSummary: {
    item: 'Doors',
    summary: { operational: 1, outOfOrder: 1 },
    outOfOrder: [],
  },
  handleClick: () => {
    /**filler */
  },
};

const itemStateDataRobot: MainMenuItemStateProps = {
  itemSummary: {
    item: 'Robots',
    summary: { operational: 2, outOfOrder: 0, idle: 1, charging: 1 },
    outOfOrder: [],
  },
  handleClick: () => {
    /**filler */
  },
};

const mainMenuAlert: MainMenuAlertProps = {
  notifications: [{ severity: 'High', time: 'January 29th 2021, 8:20:50', error: 'hello world' }],
};

const mainMenuTaskStateData: MainMenuTaskStateProps = {
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
};

const mainMenuBannerData: MainMenuBannerProps = {
  bannerUrl: 'testUrl',
  isError: false,
};

export const MainMenuItemStateStory: Story = (args) => (
  <React.Fragment>
    <MainMenuItemState
      itemSummary={itemStateDataDoor.itemSummary}
      handleClick={itemStateDataDoor.handleClick}
      {...args}
    />
    <MainMenuItemState
      itemSummary={itemStateDataRobot.itemSummary}
      handleClick={itemStateDataRobot.handleClick}
      {...args}
    />
  </React.Fragment>
);

export const MainMenuItemAlertStory: Story = (args) => (
  <React.Fragment>
    <MainMenuAlert notifications={mainMenuAlert.notifications} {...args} />
    <MainMenuAlert notifications={[]} {...args} />
  </React.Fragment>
);

export const MainMenuTaskStateStory: Story = (args) => (
  <MainMenuTaskState tasks={mainMenuTaskStateData.tasks} {...args} />
);

export const MainMenuBannerStory: Story = (args) => (
  <React.Fragment>
    <MainMenuBanner
      bannerUrl={mainMenuBannerData.bannerUrl}
      isError={mainMenuBannerData.isError}
      {...args}
    />
    <div style={{ margin: '1rem 0' }}></div>
    <MainMenuBanner
      bannerUrl={mainMenuBannerData.bannerUrl}
      isError={!mainMenuBannerData.isError}
      {...args}
    />
  </React.Fragment>
);
