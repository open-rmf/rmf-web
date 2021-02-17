import React from 'react';
import ReactDOM from 'react-dom';
import MainMenu from '../main-menu';
import { createMount } from '@material-ui/core/test-utils';
import RmfHealthStateManager from '../../../managers/rmf-health-state-manager';
import { RmfHealthContext } from '../../rmf-app/contexts';

const mount = createMount();

const healthManager = new RmfHealthStateManager();
let healthStatus = healthManager.getHealthStatus();
healthStatus = {
  door: {
    item: 'Door',
    itemSummary: { operational: 1, outOfOrder: 0 },
    spoiltDoors: [],
  },
  lift: {
    item: 'Lift',
    itemSummary: { operational: 1, outOfOrder: 0 },
    spoiltLifts: [],
  },
  dispenser: {
    item: 'Dispensers',
    itemSummary: { operational: 1, outOfOrder: 0 },
    spoiltDispensers: [],
  },
  robot: {
    item: 'Robots',
    robotSummary: {
      operational: 1,
      outOfOrder: 0,
      charging: 0,
      idle: 0,
    },
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
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(3).text(),
  ).toEqual('1/1');
});

it('it should not count spoilt equipment as operational', () => {
  healthStatus = {
    door: {
      ...healthStatus.door,
      itemSummary: { operational: 0, outOfOrder: 1 },
    },
    lift: {
      ...healthStatus.lift,
      itemSummary: { operational: 0, outOfOrder: 1 },
    },
    dispenser: {
      ...healthStatus.dispenser,
      itemSummary: { operational: 0, outOfOrder: 1 },
    },
    robot: {
      ...healthStatus.robot,
      robotSummary: {
        operational: 0,
        outOfOrder: 1,
        charging: 0,
        idle: 0,
      },
    },
  };
  const root = mount(
    <RmfHealthContext.Provider value={healthStatus}>
      <MainMenu pushView={jest.fn()} tasks={[]} notifications={[]} />
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
  healthStatus = {
    ...healthStatus,
    robot: {
      ...healthStatus.robot,
      robotSummary: {
        operational: 1,
        outOfOrder: 0,
        charging: 1,
        idle: 1,
      },
    },
  };
  const root = mount(
    <RmfHealthContext.Provider value={healthStatus}>
      <MainMenu pushView={jest.fn()} tasks={[]} notifications={[]} />
    </RmfHealthContext.Provider>,
  );
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(3).text(),
  ).toEqual('1/1');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(4).text(),
  ).toEqual('1');
  expect(
    root.find('SystemSummaryItemState').find('.MuiPaper-root').find('h6').at(5).text(),
  ).toEqual('1');
});
