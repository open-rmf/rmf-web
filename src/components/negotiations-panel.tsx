import React from 'react';
import { Typography } from '@material-ui/core';
import { makeStyles } from '@material-ui/core/styles';
import TreeView from '@material-ui/lab/TreeView';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';
import TreeItem from '@material-ui/lab/TreeItem';
import { SpotlightValue } from './spotlight-value';
import {
  NegotiationConflict,
  NegotiationStatus,
  ResolveState,
} from '../negotiation-status-manager';

import { NegotiationStatusManager, NegotiationTrajectoryResponse } from '../negotiation-status-manager';

const useStyles = makeStyles(theme => ({
  root: {
    height: 240,
    flexGrow: 1,
    maxWidth: 400,
  },
  finished: {
    backgroundColor: 'lightgreen',
  },
  rejected: {
    backgroundColor: theme.palette.warning.main,
  },
  forfeited: {
    backgroundColor: theme.palette.warning.main,
  },
  defunct: {
    backgroundColor: theme.palette.error.main,
  },
  unresolved: {
    backgroundColor: theme.palette.error.main,
  },
  ongoing: {
    backgroundColor: 'yellow',
  },
}));

interface Parameter {
  map_name: string;
  conflict_version: number;
  sequence: number[];
}

export interface NegotiationsPanelProps {
  conflicts: Readonly<Record<string, NegotiationConflict>>;
  spotlight?: Readonly<SpotlightValue<string>>;
  mapFloorLayerSorted?: Readonly<string[]>;
  negotiationStatusManager?: Readonly<NegotiationStatusManager>;
  negotiationTrajStore?: Record<string, NegotiationTrajectoryResponse>;
}

export default function NegotiationsPanel(props: NegotiationsPanelProps): JSX.Element {
  const { conflicts, spotlight, mapFloorLayerSorted, negotiationStatusManager, negotiationTrajStore } = props;

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    // TODO: spotlight
  }, [spotlight]);

  let curLevelName = "";
  if (mapFloorLayerSorted)
    curLevelName = mapFloorLayerSorted[0];

  const classes = useStyles();

  /**  Utility conversion functions **/
  const determineStatusText = (
    status: NegotiationStatus,
    parent_resolved: ResolveState,
  ): string => {
    if (status.forfeited) return '  [FORFEITED]';
    else if (status.rejected) return '  [REJECTED]';
    else if (status.defunct) return '  [DEFUNCT]';
    else if (parent_resolved === ResolveState.RESOLVED) return '  [FINISHED]';
    else return '  [ONGOING]';
  };
  const determineStyle = (status: NegotiationStatus, parent_resolved: ResolveState): string => {
    if (status.forfeited) return classes.forfeited;
    else if (status.rejected) return classes.rejected;
    else if (status.defunct) return classes.defunct;
    else if (parent_resolved === ResolveState.RESOLVED) return classes.finished;
    else return classes.ongoing;
  };

  // keep track of parameters so we can send them as requests
  let nodeidToParameters = new Map<string, Parameter>();

  /** Render Negotiation panel contents **/
  const renderNegotiations = (version: string, conflict: NegotiationConflict) => {
    let conflictLabel = 'Conflict #' + version + ', Participants: ';
    let i = 0;
    for (const name of Object.values(conflict.participantIdsToNames)) {
      conflictLabel += name;

      if (i !== Object.keys(conflict.participantIdsToNames).length - 1) conflictLabel += ', ';
      ++i;
    }
    let conflictStyle = classes.ongoing;
    if (conflict.resolved === ResolveState.RESOLVED) conflictStyle = classes.finished;
    else if (conflict.resolved === ResolveState.FAILED) conflictStyle = classes.unresolved;

    let tableDom: JSX.Element[] = [];
    for (const [participantId, statusData] of Object.entries(conflict.participantIdsToStatus)) {
      // status handling and background
      const participantName = conflict.participantIdsToNames[participantId];

      // add 1 or 2 rows of data depending on the sequence

      if (statusData.hasTerminal && statusData.terminal.sequence.length > 1) {
        const terminalStatus = statusData.terminal;

        //set text and style for terminal node
        let terminalLabelText = participantName;
        terminalLabelText += ' -> [';
        const lastIdx = terminalStatus.sequence.length - 1;
        for (let idx = 0; idx < lastIdx; ++idx) {
          let sequenceId = terminalStatus.sequence[idx].toString();
          let sequenceIdName = conflict.participantIdsToNames[sequenceId];

          terminalLabelText += sequenceIdName;
          if (idx !== lastIdx - 1) terminalLabelText += ', ';
        }
        terminalLabelText += ']';
        terminalLabelText += determineStatusText(terminalStatus, conflict.resolved);

        let terminalStyle = determineStyle(terminalStatus, conflict.resolved);

        //set text and style for base node
        let baseStatus = statusData.base;

        let baseLabelText = '[';
        const sequenceIdName = conflict.participantIdsToNames[baseStatus.sequence[0]];
        baseLabelText += sequenceIdName;
        baseLabelText += ']';
        baseLabelText += determineStatusText(baseStatus, conflict.resolved);

        let baseStyle = determineStyle(baseStatus, conflict.resolved);

        let terminalId = version + '.' + participantId + '.terminal'; //terminal node
        let baseId = version + '.' + participantId + '.base'; //base ID
        tableDom.push(
          <TreeItem
            data-component="TreeItem"
            nodeId={terminalId}
            key={terminalId}
            classes={{ label: terminalStyle, selected: terminalStyle }}
            label={terminalLabelText}
          >
            <TreeItem
              data-component="TreeItem"
              nodeId={baseId}
              key={baseId}
              classes={{ label: baseStyle, selected: baseStyle }}
              label={baseLabelText}
            />
          </TreeItem>,
        );

        let terminalParams = {
          map_name: curLevelName,
          conflict_version: parseInt(version),
          sequence: terminalStatus.sequence,
        };
        nodeidToParameters.set(terminalId, terminalParams);

        let baseParams = {
          map_name: curLevelName,
          conflict_version: parseInt(version),
          sequence: baseStatus.sequence,
        };
        nodeidToParameters.set(baseId, baseParams);
      } else {
        //single node
        let baseStatus = statusData.base;
        let labelText = participantName + determineStatusText(baseStatus, conflict.resolved);
        let style = determineStyle(baseStatus, conflict.resolved);
        let nodeId = version + '.' + participantId + '.base'; //base ID
        tableDom.push(
          <TreeItem
            data-component="TreeItem"
            nodeId={nodeId}
            key={nodeId}
            classes={{ label: style, selected: style }}
            label={labelText}
          />,
        );

        let baseParams = {
          map_name: curLevelName,
          conflict_version: parseInt(version),
          sequence: baseStatus.sequence,
        };
        nodeidToParameters.set(nodeId, baseParams);
      }
    }

    let nodeIdBase = 'conflict' + version;
    return (
      <TreeItem
        data-component="TreeItem"
        nodeId={nodeIdBase}
        key={nodeIdBase}
        classes={{ label: conflictStyle, selected: conflictStyle }}
        label={conflictLabel}
      >
        {tableDom}
      </TreeItem>
    );
  };

  let negotiationContents: JSX.Element[] = [];
  if (conflicts) {
    let reversedConflicts = Object.keys(conflicts).reverse();
    reversedConflicts.forEach(version => {
      const conflict = conflicts[version];
      let contents = renderNegotiations(version, conflict);
      negotiationContents.push(contents);
    });
  } else {
    console.log('prop negotationstatus empty');
  }

  // action callbacks
  const handleSelect = (event: React.ChangeEvent<{}>, nodeIds: string): void => {
    async function updateNegotiationTrajectory() {
      if (!negotiationStatusManager || !negotiationTrajStore) {
        return;
      }

      const trajParams = nodeidToParameters.get(nodeIds);
      if (!trajParams) {
        // Must have clicked a top level node
        return;
      }

      const resp = await negotiationStatusManager.negotiationTrajectory({
        request: 'negotiation_trajectory',
        param: trajParams,
      });
      if (resp.values === undefined) console.warn('values undefined!');
      if (resp.response !== 'negotiation_trajectory') {
        console.warn('wrong response, ignoring!');
        return;
      }
      
      negotiationTrajStore[trajParams.map_name] = resp;
    }

    updateNegotiationTrajectory();
  };

  return (
    <Typography variant="body1" component={'span'} >
      <TreeView
        onNodeSelect={handleSelect}
        defaultCollapseIcon={<ExpandMoreIcon />}
        defaultExpanded={['root']}
        defaultExpandIcon={<ChevronRightIcon />}
      >
        {negotiationContents}
      </TreeView>
    </Typography>
  );
}
