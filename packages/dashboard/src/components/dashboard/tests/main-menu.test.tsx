import React from 'react';
import ReactDOM from 'react-dom';
import MainMenu from '../main-menu';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { createMount } from '@material-ui/core/test-utils';
import RmfHealthStateManager, { ItemState } from '../../../managers/rmf-health-state-manager';
import { RmfHealthContext } from '../../rmf-app/contexts';

const mount = createMount();

const mockItemState: ItemState = {
  doors: {
    testDoor: {
      door_name: 'testDoor',
      door_time: { sec: 0, nanosec: 0 },
      current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
    },
  },
  dispensers: {
    testDispenser: {
      time: { sec: 0, nanosec: 0 },
      guid: 'testDispenser',
      mode: RomiCore.DispenserState.IDLE,
      request_guid_queue: [],
      seconds_remaining: 0,
    },
  },
  lifts: {
    testLift: {
      lift_name: 'testLift',
      lift_time: { sec: 0, nanosec: 0 },
      available_floors: [],
      current_floor: '',
      destination_floor: '',
      door_state: RomiCore.DoorMode.MODE_OPEN,
      motion_state: RomiCore.LiftState.MOTION_STOPPED,
      available_modes: new Uint8Array(),
      current_mode: RomiCore.LiftState.DOOR_OPEN,
      session_id: '',
    },
  },
  robots: {
    fleet: {
      name: 'fleet',
      robots: [
        {
          name: 'testRobot',
          model: 'model',
          task_id: '',
          battery_percent: 20,
          mode: { mode: RomiCore.RobotMode.MODE_MOVING },
          location: {
            level_name: 'L1',
            x: 0,
            y: 0,
            yaw: 0,
            t: { sec: 0, nanosec: 0 },
          },
          path: [],
        },
      ],
    },
  },
};

it('renders without crashing', () => {
  const div = document.createElement('div');
  const healthManager = new RmfHealthStateManager(mockItemState);
  ReactDOM.render(
    <RmfHealthContext.Provider value={healthManager.getHealthStatus()}>
      <MainMenu
        pushView={jest.fn()}
        tasks={[]}
        notifications={[]}
        doors={[]}
        dispensers={{}}
        lifts={[]}
        robots={{}}
      />
      ,
    </RmfHealthContext.Provider>,
    div,
  );
  ReactDOM.unmountComponentAtNode(div);
});

it('should count working equipment as operational', () => {
  const healthManager = new RmfHealthStateManager(mockItemState);
  const root = mount(
    <RmfHealthContext.Provider value={healthManager.getHealthStatus()}>
      <MainMenu
        pushView={jest.fn()}
        tasks={[]}
        notifications={[]}
        doors={[]}
        dispensers={{}}
        lifts={[]}
        robots={{}}
      />
    </RmfHealthContext.Provider>,
  );
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(0).text(),
  ).toEqual('1/1');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(1).text(),
  ).toEqual('1/1');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(2).text(),
  ).toEqual('1/1');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(3).text(),
  ).toEqual('1/1');
});

it('it should not count spoilt equipment as operational', () => {
  const mockSpoiltItem: ItemState = {
    doors: {
      testDoor: { ...mockItemState.doors.testDoor, current_mode: { value: 10 } },
    },
    dispensers: {
      testDispenser: {
        ...mockItemState.dispensers.testDispenser,
        mode: 10,
      },
    },
    lifts: {
      testLift: {
        ...mockItemState.lifts.testLift,
        current_mode: RomiCore.LiftState.MODE_FIRE,
      },
    },
    robots: {
      fleet: {
        name: 'fleet',
        robots: [
          {
            ...mockItemState.robots.fleet.robots[0],
            mode: { mode: RomiCore.RobotMode.MODE_ADAPTER_ERROR },
          },
        ],
      },
    },
  };
  const healthManager = new RmfHealthStateManager(mockSpoiltItem);
  const root = mount(
    <RmfHealthContext.Provider value={healthManager.getHealthStatus()}>
      <MainMenu
        pushView={jest.fn()}
        tasks={[]}
        notifications={[]}
        doors={[]}
        dispensers={{}}
        lifts={[]}
        robots={{}}
      />
    </RmfHealthContext.Provider>,
  );
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(0).text(),
  ).toEqual('0/1');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(1).text(),
  ).toEqual('0/1');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(2).text(),
  ).toEqual('0/1');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(3).text(),
  ).toEqual('0/1');
});

it('should count robots that are charging and on idle as operational', () => {
  const idleAndChargingRobots = {
    ...mockItemState,
    robots: {
      fleet: {
        name: 'fleet',
        robots: [
          {
            ...mockItemState.robots.fleet.robots[0],
            mode: { mode: RomiCore.RobotMode.MODE_CHARGING },
          },
          {
            ...mockItemState.robots.fleet.robots[0],
            mode: { mode: RomiCore.RobotMode.MODE_IDLE },
          },
        ],
      },
    },
  };
  const healthManager = new RmfHealthStateManager(idleAndChargingRobots);
  const root = mount(
    <RmfHealthContext.Provider value={healthManager.getHealthStatus()}>
      <MainMenu
        pushView={jest.fn()}
        tasks={[]}
        notifications={[]}
        doors={[]}
        dispensers={{}}
        lifts={[]}
        robots={{}}
      />
    </RmfHealthContext.Provider>,
  );
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(4).text(),
  ).toEqual('1');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(5).text(),
  ).toEqual('1');
});
