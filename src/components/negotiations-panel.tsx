import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
//import DispenserItem from './dispenser-item';
import { makeStyles } from '@material-ui/core/styles';
import TreeView from '@material-ui/lab/TreeView';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';
import TreeItem from '@material-ui/lab/TreeItem';
import { SpotlightValue } from './spotlight-value';
import { NegotiationStatus, negotiationStatus } from '@osrf/romi-js-core-interfaces';
import * as NegotiationStatusManager from '../negotiation-status-manager';
import {
  RobotTrajectoryManager,
  NegotiationTrajectoryRequest,
  NegotiationTrajectoryResponse
} from '../robot-trajectory-manager';
import { throws } from 'assert';
import { trajectoryPath } from './schedule-visualizer/robot-trajectory';

const useStyles = makeStyles({
  root: {
    height: 240,
    flexGrow: 1,
    maxWidth: 400,
  },
  finished: {
    backgroundColor: "lightgreen"
  },
  rejected: {
    backgroundColor: "red"
  },
  ongoing: {
    backgroundColor: "yellow"
  }
});

interface Parameter
{
  conflict_version : number;
  sequence : number[];
};

function toNumberArray(arr : Uint32Array) : number[] {
  var ret : number[] = [];
  arr.forEach(value => {
    ret.push(value);
  });
  return ret;
}

export interface NegotiationsPanelProps {
  conflicts : Readonly<Record<string, NegotiationStatusManager.NegotiationConflict>>;
  spotlight?: Readonly<SpotlightValue<string>>;
  trajManager?: Readonly<RobotTrajectoryManager>;
  negotiationTrajStore : Record<string, NegotiationTrajectoryResponse>;
}

export default function NegotiationsPanel(props: NegotiationsPanelProps): JSX.Element {
  const { spotlight } = props;

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    /*const ref = dispenserRefs.current[spotlight.value];
    if (!ref) {
      return;
    }
    setExpanded(prev => ({
      ...prev,
      [spotlight.value]: true,
    }));
    ref.scrollIntoView({ behavior: 'smooth' });
    */
  }, [spotlight]);
  
  
  const classes = useStyles();

  let rejected_statuses = NegotiationStatus.STATUS_REJECTED | 
        NegotiationStatus.STATUS_FORFEITED |
        NegotiationStatus.STATUS_DEFUNCT;

  const addStatusText = (status : number) => {
    var status_text = "";
    if (status & NegotiationStatus.STATUS_FINISHED)
      status_text += "  [FINISHED]";
    else if (status & rejected_statuses)
      status_text += "  [FAILED]";
    else
      status_text += "  [ONGOING]";
    return status_text;
  };

  var nodeid_to_parameters = new Map<string, Parameter>();

  const renderNegotiations = (version : string, conflict : NegotiationStatusManager.NegotiationConflict) => {
    let conflict_label = "Conflict #" + version + ", Participants: ";
    var i = 0;
    for (var [participant_id, name] of Object.entries(conflict.participant_ids_to_names))
    {
      conflict_label += name;

      if (i !== (Object.keys(conflict.participant_ids_to_names).length - 1))
        conflict_label += ", ";
      ++i;
    }
    let conflict_style = classes.ongoing;
    if (conflict.resolved === NegotiationStatusManager.ResolveState.RESOLVED)
      conflict_style = classes.finished;
    else if (conflict.resolved === NegotiationStatusManager.ResolveState.NOT_RESOLVED)
      conflict_style = classes.finished;

    var table_dom : JSX.Element[] = [];
    for (var [participant_id, status] of Object.entries(conflict.participant_ids_to_status))
    {
      // status handling and background
      let style = classes.ongoing;
      if (status.status & NegotiationStatus.STATUS_FINISHED)
        style = classes.finished;
      else if (status.status & rejected_statuses)
        style = classes.rejected;
        
      var participant_name = conflict.participant_ids_to_names[participant_id];
        
      //add 1 or 2 rows of data depending on the sequence
      let label_text = participant_name;
      if (status.sequence.length > 1)
      {
        label_text += " -> [";
        var last_idx = (status.sequence.length - 1);
        for (var i = 0; i < last_idx; ++i)
        {
          var seq_id = status.sequence[i].toString();
          var seq_id_name = conflict.participant_ids_to_names[seq_id];

          label_text += seq_id_name;
          if (i != (last_idx - 1))
            label_text += ", ";
        }
        label_text += "]";
        label_text += addStatusText(status.status);
        

        let label_text2 = "[";
        var seq_id_name = conflict.participant_ids_to_names[status.sequence[last_idx]];
        label_text2 += seq_id_name;
        label_text2 += "]";
        label_text2 += addStatusText(status.status);

        var nodeId = version + "." + participant_id + ".terminal"; //terminal node
        var childId = version + "." + participant_id + ".base"; //base ID
        table_dom.push(
          <TreeItem nodeId={nodeId} key={nodeId} classes={{label: style, selected: style}} label={label_text}>
            <TreeItem nodeId={childId} key={childId} classes={{label: style, selected: style}} label={label_text2}/>
          </TreeItem>);
        
        let terminal_params = {
            conflict_version : parseInt(version),
            sequence : toNumberArray(status.sequence)
          };
        nodeid_to_parameters.set(nodeId, terminal_params);

        let base_params = {
          conflict_version : parseInt(version),
          sequence : [parseInt(participant_id)]
        };
        nodeid_to_parameters.set(childId, base_params);
      }
      else
      {
        //single node
        label_text += addStatusText(status.status);
        var nodeId = version + "." + participant_id + ".base"; //base ID
        table_dom.push(
          <TreeItem nodeId={nodeId} key={nodeId} classes={{label: style, selected: style}} label={label_text}/>
        );

        let base_params = {
          conflict_version : parseInt(version),
          sequence : [parseInt(participant_id)]
        };
        nodeid_to_parameters.set(nodeId, base_params);
      }
    }
    
    var nodeIdBase = "conflict" + version;
    return (
      <TreeItem nodeId={nodeIdBase} key={nodeIdBase} classes={{ label: conflict_style, selected: conflict_style }} label={conflict_label}>
        { table_dom }
      </TreeItem>
    );
  };

  let negotiation_contents : JSX.Element[] = [];
  if (props.conflicts) {
    var reversed_conflicts = Object.keys(props.conflicts).reverse()
    reversed_conflicts.map(version => {
      const conflict = props.conflicts[version];
      let contents = renderNegotiations(version, conflict);
      negotiation_contents.push(contents);
    });
  }
  else {
    console.log("prop negotationstatus empty");
  }
  
  // action callbacks
  const handleSelect = (event: React.ChangeEvent<{}>, nodeIds: string) => {
    //console.log("selected: " + nodeIds);

    async function updateNegotiationTrajectory() {
      if (!props.trajManager) {
        return;
      }

      var traj_params = nodeid_to_parameters.get(nodeIds);
      if (!traj_params) {
        //Must have clicked a top level node
        return;
      }

      const resp = await props.trajManager.negotiationTrajectory({ 
        request: 'negotiation_trajectory',
        param: traj_params
      });
      if (resp.values == undefined)
        console.warn("values undefined!");

      resp.values.forEach(value => {
        console.log("id: " + value.id);
      })
      props.negotiationTrajStore["L1"] = resp;
    };;

    updateNegotiationTrajectory();
    //interval = window.setInterval(updateNegotiationTrajectory, trajAnimDuration);
  };

  return (
    <TreeView
      className={classes.root}
      onNodeSelect={handleSelect}
      defaultCollapseIcon={<ExpandMoreIcon />}
      defaultExpanded={['root']}
      defaultExpandIcon={<ChevronRightIcon />}
    >
      {negotiation_contents}
    </TreeView>
  );
}
