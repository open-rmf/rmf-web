import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import TreeView from '@material-ui/lab/TreeView';
import TreeItem from '@material-ui/lab/TreeItem';
import { NegotiationConflict, ResolveState } from '../../negotiation-status-manager';
import NegotiationsPanel from '../negotiations-panel';

const mount = createMount();

let negotiationStatuses : Record<number, NegotiationConflict>;

beforeEach(() => {
  negotiationStatuses = {
    0: {
      participantIdsToNames: { 
        "1":"tinyrobot1",
        "2":"tinyrobot2" 
      },
      participantIdsToStatus: { 
        "1": {
          hasTerminal: true,
          base: { 
            sequence: [1],
            defunct: false,
            rejected: true,
            forfeited: false 
          },
          terminal: { 
            sequence: [2, 1],
            defunct: false,
            rejected: false,
            forfeited: false 
          }
        },
        "2": {
          hasTerminal: false,
          base: { 
            sequence: [2],
            defunct: false,
            rejected: false,
            forfeited: true 
          },
          terminal: {
            sequence: [1, 2],
            defunct: true,
            rejected: false,
            forfeited: false 
          }
        },
      },
      resolved: ResolveState.RESOLVED
    }
  }
});

it('renders negotiations', () => {
  const root = mount(<NegotiationsPanel 
    conflicts={negotiationStatuses}
    spotlight={undefined}
    negotiationStatusManager={undefined}
    negotiationTrajStore={undefined} />);

  const treeView = root.find(TreeView);
  expect(treeView).toBeDefined();
  
  const treeItem = root.find(TreeItem);
  expect(treeItem).toBeDefined();

  expect(root.html()).toMatchSnapshot();

  root.unmount();
});
