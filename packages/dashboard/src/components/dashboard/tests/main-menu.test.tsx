import React from 'react';
import ReactDOM from 'react-dom';
import MainMenu from '../main-menu';
import { createMount } from '@material-ui/core/test-utils';
import RmfHealthStateManager from '../../../managers/rmf-health-state-manager';
import { RmfHealthContext } from '../../rmf-app/contexts';
import { door, lift, fakeFleets } from './items';

const mount = createMount();
const fleet = fakeFleets();

const healthManager = new RmfHealthStateManager();
let healthStatus = healthManager.getHealthStatus();
healthStatus = {
  door: {
    operational: 1,
    spoiltItem: [],
  },
  lift: {
    operational: 1,
    spoiltItem: [],
  },
  dispenser: {
    operational: 1,
    spoiltItem: [],
  },
  robot: {
    operational: 1,
    charging: 0,
    idle: 0,
    spoiltRobots: [],
  },
};

it('renders without crashing', () => {
  const div = document.createElement('div');
  ReactDOM.render(
    <RmfHealthContext.Provider value={healthStatus}>
      <MainMenu pushView={jest.fn()} tasks={[]} notifications={[]} />,
    </RmfHealthContext.Provider>,
    div,
  );
  ReactDOM.unmountComponentAtNode(div);
});

it('should count working equipment as operational', () => {
  const root = mount(
    <RmfHealthContext.Provider value={healthStatus}>
      <MainMenu pushView={jest.fn()} tasks={[]} notifications={[]} />
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
  expect(root.find('RobotSummaryState').find('.MuiPaper-root').find('h6').at(0).text()).toEqual(
    '1/1',
  );
});

it('it should not count spoilt equipment as operational', () => {
  healthStatus = {
    door: {
      ...healthStatus.door,
      spoiltItem: [{ name: 'name', state: 'state', door: door }],
    },
    lift: {
      ...healthStatus.lift,
      spoiltItem: [{ name: 'name', state: 'state', lift: lift }],
    },
    dispenser: {
      ...healthStatus.dispenser,
      spoiltItem: [{ name: 'name', state: 'state', dispenser: 'dispenser' }],
    },
    robot: {
      operational: 1,
      charging: 0,
      idle: 0,
      spoiltRobots: [{ fleet: 'fleet', name: 'robot', state: 'state', robot: fleet[0].robots[0] }],
    },
  };
  const root = mount(
    <RmfHealthContext.Provider value={healthStatus}>
      <MainMenu pushView={jest.fn()} tasks={[]} notifications={[]} />
    </RmfHealthContext.Provider>,
  );
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(0).text(),
  ).toEqual('1/2');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(1).text(),
  ).toEqual('1/2');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(2).text(),
  ).toEqual('1/2');
  expect(root.find('RobotSummaryState').find('.MuiPaper-root').find('h6').at(0).text()).toEqual(
    '1/2',
  );
});

it('should count robots that are charging and on idle as operational', () => {
  healthStatus = {
    ...healthStatus,
    robot: {
      operational: 2,
      charging: 1,
      idle: 1,
      spoiltRobots: [],
    },
  };
  const root = mount(
    <RmfHealthContext.Provider value={healthStatus}>
      <MainMenu pushView={jest.fn()} tasks={[]} notifications={[]} />
    </RmfHealthContext.Provider>,
  );
  expect(root.find('RobotSummaryState').find('.MuiPaper-root').find('h6').at(0).text()).toEqual(
    '2/2',
  );
  expect(root.find('RobotSummaryState').find('.MuiPaper-root').find('h6').at(1).text()).toEqual(
    '1',
  );
  expect(root.find('RobotSummaryState').find('.MuiPaper-root').find('h6').at(2).text()).toEqual(
    '1',
  );
});
