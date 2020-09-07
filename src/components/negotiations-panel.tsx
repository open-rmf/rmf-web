import React from 'react';
import { makeStyles } from '@material-ui/core/styles';
import TreeView from '@material-ui/lab/TreeView';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';
import TreeItem from '@material-ui/lab/TreeItem';
import { SpotlightValue } from './spotlight-value';
import { 
  NegotiationConflict,
  NegotiationStatus,
  ResolveState
} from '../negotiation-status-manager';

import {
  RobotTrajectoryManager,
  NegotiationTrajectoryResponse
} from '../robot-trajectory-manager';

const useStyles = makeStyles(theme => ({
  root: {
    height: 240,
    flexGrow: 1,
    maxWidth: 400,
  },
  finished: {
    backgroundColor: "lightgreen"
  },
  rejected: {
    backgroundColor: theme.palette.warning.main
  },
  forfeited: {
    backgroundColor: theme.palette.warning.main
  },
  defunct: {
    backgroundColor: theme.palette.error.main
  },
  unresolved: {
    backgroundColor: theme.palette.error.main
  },
  ongoing: {
    backgroundColor: "yellow"
  }
}));

interface Parameter
{
  conflict_version : number;
  sequence : number[];
};

export interface NegotiationsPanelProps {
  conflicts : Readonly<Record<string, NegotiationConflict>>;
  spotlight?: Readonly<SpotlightValue<string>>;
  trajManager?: Readonly<RobotTrajectoryManager>;
  negotiationTrajStore : Record<string, NegotiationTrajectoryResponse>;
}

export default function NegotiationsPanel(props: NegotiationsPanelProps): JSX.Element {
  const { conflicts, spotlight, trajManager, negotiationTrajStore } = props;

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    // TODO: spotlight
  }, [spotlight]);
  
  const classes = useStyles();

  /**  Utility conversion functions **/
  const determineStatusText = (status : NegotiationStatus,
    parent_resolved : ResolveState) => {
    let status_text = "";
    if (status.forfeited)
      status_text += "  [FORFEITED]";
    else if (status.rejected)
      status_text += "  [REJECTED]";
    else if (status.defunct)
      status_text += "  [DEFUNCT]";
    else if (parent_resolved === ResolveState.RESOLVED)
      status_text += "  [FINISHED]";
    else
      status_text += "  [ONGOING]";
    return status_text;
  };
  const determineStyle = (status : NegotiationStatus, 
    parent_resolved : ResolveState) => {
    let style = classes.ongoing;
    if (status.forfeited)
      style = classes.forfeited;
    else if (status.rejected)
      style = classes.rejected;
    else if (status.defunct)
      style = classes.defunct;
    else if (parent_resolved === ResolveState.RESOLVED)
      style = classes.finished;
    return style;
  };

  // keep track of parameters so we can send them as requests
  let nodeid_to_parameters = new Map<string, Parameter>();

  /** Render Negotiation panel contents **/
  const renderNegotiations = (version : string, conflict : NegotiationConflict) => {
    let conflict_label = "Conflict #" + version + ", Participants: ";
    let i = 0;
    for (const name of Object.values(conflict.participant_ids_to_names))
    {
      conflict_label += name;

      if (i !== (Object.keys(conflict.participant_ids_to_names).length - 1))
        conflict_label += ", ";
      ++i;
    }
    let conflict_style = classes.ongoing;
    if (conflict.resolved === ResolveState.RESOLVED)
      conflict_style = classes.finished;
    else if (conflict.resolved === ResolveState.FAILED)
      conflict_style = classes.unresolved;

    let table_dom : JSX.Element[] = [];
    for (const [participant_id, status_data] of Object.entries(conflict.participant_ids_to_status))
    {
      // status handling and background
      const participant_name = conflict.participant_ids_to_names[participant_id];
        
      //add 1 or 2 rows of data depending on the sequence
      
      if (status_data.has_terminal && status_data.terminal.sequence.length > 1)
      {
        const terminal_status = status_data.terminal;
        
        //set text and style for terminal node
        let terminal_label_text = participant_name;
        terminal_label_text += " -> [";
        const last_idx = (terminal_status.sequence.length - 1);
        for (let idx = 0; idx < last_idx; ++idx)
        {
          let seq_id = terminal_status.sequence[idx].toString();
          let seq_id_name = conflict.participant_ids_to_names[seq_id];

          terminal_label_text += seq_id_name;
          if (idx !== (last_idx - 1))
            terminal_label_text += ", ";
        }
        terminal_label_text += "]";
        terminal_label_text += determineStatusText(terminal_status, conflict.resolved);

        let terminal_style = determineStyle(terminal_status, conflict.resolved);

        //set text and style for base node
        let base_status = status_data.base;

        let base_label_text = "[";
        const seq_id_name = conflict.participant_ids_to_names[base_status.sequence[0]];
        base_label_text += seq_id_name;
        base_label_text += "]";
        base_label_text += determineStatusText(base_status, conflict.resolved);

        let base_style = determineStyle(base_status, conflict.resolved);

        let terminalId = version + "." + participant_id + ".terminal"; //terminal node
        let baseId = version + "." + participant_id + ".base"; //base ID
        table_dom.push(
          <TreeItem nodeId={terminalId} key={terminalId} classes={{label: terminal_style, selected: terminal_style}} label={terminal_label_text}>
            <TreeItem nodeId={baseId} key={baseId} classes={{label: base_style, selected: base_style}} label={base_label_text}/>
          </TreeItem>);
        
        let terminal_params = {
            conflict_version : parseInt(version),
            sequence : terminal_status.sequence
          };
        nodeid_to_parameters.set(terminalId, terminal_params);

        let base_params = {
          conflict_version : parseInt(version),
          sequence : base_status.sequence
        };
        nodeid_to_parameters.set(baseId, base_params);
      }
      else
      {
        //single node
        let base_status = status_data.base;
        let label_text = participant_name + determineStatusText(base_status, conflict.resolved);
        let style = determineStyle(base_status, conflict.resolved);
        let nodeId = version + "." + participant_id + ".base"; //base ID
        table_dom.push(
          <TreeItem nodeId={nodeId} key={nodeId} classes={{label: style, selected: style}} label={label_text}/>
        );

        let base_params = {
          conflict_version : parseInt(version),
          sequence : base_status.sequence
        };
        nodeid_to_parameters.set(nodeId, base_params);
      }
    }
    
    let nodeIdBase = "conflict" + version;
    return (
      <TreeItem nodeId={nodeIdBase} key={nodeIdBase} classes={{ label: conflict_style, selected: conflict_style }} label={conflict_label}>
        { table_dom }
      </TreeItem>
    );
  };

  let negotiation_contents : JSX.Element[] = [];
  if (conflicts) {
    let reversed_conflicts = Object.keys(conflicts).reverse();
    reversed_conflicts.forEach(version => {
      const conflict = conflicts[version];
      let contents = renderNegotiations(version, conflict);
      negotiation_contents.push(contents);
    });
  }
  else {
    console.log("prop negotationstatus empty");
  }
  
  // action callbacks
  const handleSelect = (event: React.ChangeEvent<{}>, nodeIds: string) :void => {

    async function updateNegotiationTrajectory() {
      if (!trajManager) {
        return;
      }

      const traj_params = nodeid_to_parameters.get(nodeIds);
      if (!traj_params) {
        //Must have clicked a top level node
        return;
      }

      const resp = await trajManager.negotiationTrajectory({ 
        request: 'negotiation_trajectory',
        param: traj_params
      });
      if (resp.values === undefined)
        console.warn("values undefined!");

      negotiationTrajStore["L1"] = resp;
    };

    updateNegotiationTrajectory();
  };

  return (
    <TreeView
      onNodeSelect={handleSelect}
      defaultCollapseIcon={<ExpandMoreIcon />}
      defaultExpanded={['root']}
      defaultExpandIcon={<ChevronRightIcon />}
    >
      {negotiation_contents}
    </TreeView>
  );
}
