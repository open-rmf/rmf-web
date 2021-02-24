import React from 'react';
import {
  SystemSummaryItemState,
  SystemSummaryAlertProps,
  SystemSummaryAlert,
  SystemSummaryTaskState,
  SystemSummaryTaskStateProps,
  SystemSummaryBanner,
  SystemSummaryBannerProps,
  SystemSummarySpoiltItems,
  SystemSummarySpoiltItemsProps,
  ItemSummary,
  RobotSummary,
  Severity,
  RobotSummaryState,
  OmniPanel,
  OmniPanelView,
  StackNavigator,
} from '../lib';
import { Meta, Story } from '@storybook/react';
import { Button } from '@material-ui/core';

export default {
  title: 'Systems summary',
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

const SimpleItemSummary = (): JSX.Element => {
  const [view, setView] = React.useState<number | string>(0);
  const stack = React.useMemo(() => new StackNavigator<number>(0), []);

  return (
    <OmniPanel
      view={view}
      style={{
        width: 1000,
        height: 500,
        border: '1px solid black',
        borderTopLeftRadius: 16,
        borderTopRightRadius: 16,
      }}
      onBack={() => setView(stack.pop())}
      onHome={() => setView(stack.reset())}
    >
      <OmniPanelView viewId={0}>
        <SystemSummaryItemState
          item={'Door'}
          itemSummary={itemSummary}
          onClick={() => (stack.push(1), setView(1))}
        />

        <RobotSummaryState
          item={'Robots'}
          itemSummary={robotSummary}
          onClick={() => (stack.push(2), setView(2))}
        />
      </OmniPanelView>
      <OmniPanelView viewId={1}>
        <Button variant="outlined" onClick={() => setView(stack.pop())}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView viewId={2}>
        <Button variant="outlined" onClick={() => setView(stack.pop())}>
          Back
        </Button>
      </OmniPanelView>
    </OmniPanel>
  );
};

const SimpleNotifications = (): JSX.Element => {
  const [allNotifications, setAllNotifications] = React.useState(systemSummaryAlert.notifications);

  const handleDismissNotification = (id: number) => {
    const filteredNotifications = allNotifications.filter((n) => n.id !== id);
    setAllNotifications(filteredNotifications);
  };

  return (
    <SystemSummaryAlert
      notifications={allNotifications}
      onNotificationsDismiss={handleDismissNotification}
    />
  );
};

const SpoiltItem = (): JSX.Element => {
  const [view, setView] = React.useState<number | string>(0);
  const stack = React.useMemo(() => new StackNavigator<number>(0), []);

  return (
    <OmniPanel
      view={view}
      style={{
        width: 1000,
        height: 500,
        border: '1px solid black',
        borderTopLeftRadius: 16,
        borderTopRightRadius: 16,
      }}
      onBack={() => setView(stack.pop())}
      onHome={() => setView(stack.reset())}
    >
      <OmniPanelView viewId={0}>
        <SystemSummarySpoiltItems
          doors={spoiltEquipment.doors}
          lifts={spoiltEquipment.lifts}
          dispensers={spoiltEquipment.dispensers}
          robots={spoiltEquipment.robots}
          onClickSpoiltDoor={() => (stack.push(1), setView(1))}
        />
      </OmniPanelView>
      <OmniPanelView viewId={1}>
        <Button variant="outlined" onClick={() => setView(stack.pop())}>
          Back
        </Button>
      </OmniPanelView>
    </OmniPanel>
  );
};

export const SystemSummaryItemStateStory: Story = (args) => <SimpleItemSummary {...args} />;

export const SystemSummaryItemAlertStory: Story = (args) => (
  <React.Fragment>
    <SimpleNotifications {...args} />
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

export const SystemSummarySpoiltItemStory: Story = (args) => <SpoiltItem {...args} />;
