import { createMount, createShallow } from '@material-ui/core/test-utils';
import React from 'react';
import TreeView from '@material-ui/lab/TreeView';
import TreeItem from '@material-ui/lab/TreeItem';
import {
  NegotiationStatusManager,
  NegotiationConflict,
  ResolveState,
} from '../../negotiation-status-manager';
import NegotiationsPanel from '../negotiations-panel';
import toJson from 'enzyme-to-json';

const mount = createMount();
const shallow = createShallow();

let negotiationStatuses: Record<number, NegotiationConflict>;
let setNegotiationTrajStore: jest.Mock<any, any>;

beforeEach(() => {
  setNegotiationTrajStore = jest.fn();
  negotiationStatuses = {
    0: {
      participantIdsToNames: {
        '1': 'tinyrobot1',
        '2': 'tinyrobot2',
      },
      participantIdsToStatus: {
        '1': {
          hasTerminal: true,
          base: {
            sequence: [1],
            defunct: false,
            rejected: true,
            forfeited: false,
          },
          terminal: {
            sequence: [2, 1],
            defunct: false,
            rejected: false,
            forfeited: false,
          },
        },
        '2': {
          hasTerminal: false,
          base: {
            sequence: [2],
            defunct: false,
            rejected: false,
            forfeited: true,
          },
          terminal: {
            sequence: [1, 2],
            defunct: true,
            rejected: false,
            forfeited: false,
          },
        },
      },
      resolved: ResolveState.RESOLVED,
    },
  };
});

it('renders negotiations correctly', () => {
  const root = shallow(
    <NegotiationsPanel
      conflicts={negotiationStatuses}
      spotlight={undefined}
      mapFloorLayerSorted={undefined}
      negotiationStatusManager={undefined}
      negotiationTrajStore={undefined}
      negotiationStatusUpdateTS={0}
      setNegotiationTrajStore={setNegotiationTrajStore}
    />,
  );

  const treeView = root.find(TreeView);
  expect(treeView).toBeDefined();

  const treeItem = root.find(TreeItem);
  expect(treeItem).toBeDefined();
  expect(treeItem).toHaveLength(4);

  {
    const label = treeItem.at(0).prop('label');
    expect(label).toBeDefined();
    expect(label).toContain('Conflict');

    const classes = treeItem.at(0).prop('classes');
    expect(classes).toBeDefined();
    expect(classes?.label?.includes('finished'));
  }

  {
    const label = treeItem.at(1).prop('label');
    expect(label).toBeDefined();
    expect(label).toContain('[FINISHED]');

    const classes = treeItem.at(1).prop('classes');
    expect(classes).toBeDefined();
    expect(classes?.label?.includes('finished'));
  }

  {
    const label = treeItem.at(2).prop('label');
    expect(label).toBeDefined();
    expect(label).toContain('[REJECTED]');

    const classes = treeItem.at(2).prop('classes');
    expect(classes).toBeDefined();
    expect(classes?.label?.includes('rejected'));
  }

  {
    const label = treeItem.at(3).prop('label');
    expect(label).toBeDefined();
    expect(label).toContain('[FORFEITED]');

    const classes = treeItem.at(3).prop('classes');
    expect(classes).toBeDefined();
    expect(classes?.label?.includes('forfeited'));
  }

  root.unmount();
});

it('matches snapshot', () => {
  const root = mount(
    <NegotiationsPanel
      conflicts={negotiationStatuses}
      spotlight={undefined}
      mapFloorLayerSorted={undefined}
      negotiationStatusManager={undefined}
      negotiationTrajStore={undefined}
      negotiationStatusUpdateTS={0}
      setNegotiationTrajStore={setNegotiationTrajStore}
    />,
  );

  expect(toJson(root)).toMatchSnapshot();

  root.unmount();
});

it('tests negotiation status manager', () => {
  let negotiationStatusManager: NegotiationStatusManager;
  negotiationStatusManager = new NegotiationStatusManager('');

  let conflicts = negotiationStatusManager.allConflicts();
  expect(conflicts).toBeDefined();

  negotiationStatusManager.removeOldConflicts();
  let conflicts2 = negotiationStatusManager.allConflicts();
  expect(conflicts2).toBeDefined();
});
