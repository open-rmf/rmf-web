import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
//import DispenserItem from './dispenser-item';
import { makeStyles } from '@material-ui/core/styles';
import TreeView from '@material-ui/lab/TreeView';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';
import TreeItem from '@material-ui/lab/TreeItem';
import { SpotlightValue } from './spotlight-value';
import { NegotiationStatusTable, negotiationStatus } from '@osrf/romi-js-core-interfaces';
import * as NegotiationStatusManager from '../negotiation-status-manager';

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

export interface NegotiationsPanelProps {
  conflicts : Readonly<Record<string, NegotiationStatusManager.NegotiationConflict>>;
  spotlight?: Readonly<SpotlightValue<string>>;
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

  let rejected_statuses = NegotiationStatusTable.STATUS_REJECTED | 
        NegotiationStatusTable.STATUS_FORFEITED |
        NegotiationStatusTable.STATUS_DEFUNCT;

  const addStatusText = (status : number) => {
    var status_text = "";
    if (status & NegotiationStatusTable.STATUS_FINISHED)
      status_text += "  [FINISHED]";
    else if (status & rejected_statuses)
      status_text += "  [FAILED]";
    else
      status_text += "  [ONGOING]";
    return status_text;
  };

  var key_idx = 0;
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
      if (status.status & NegotiationStatusTable.STATUS_FINISHED)
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

        var child_idx = key_idx + 1;
        table_dom.push(
          <TreeItem nodeId={key_idx.toString()} key={key_idx} classes={{label: style, selected: style}} label={label_text}>
            <TreeItem nodeId={child_idx.toString()} key={child_idx} classes={{label: style, selected: style}} label={label_text2}/>
          </TreeItem>);
        key_idx += 2;
      }
      else
      {
        label_text += addStatusText(status.status);
        //single node
        table_dom.push(
          <TreeItem nodeId={key_idx.toString()} key={key_idx} classes={{label: style, selected: style}} label={label_text}/>
        );
        key_idx += 1;
      }
    }
    
    return (
      <TreeItem nodeId={key_idx.toString()} key={key_idx} classes={{ label: conflict_style, selected: conflict_style }} label={conflict_label}>
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
      key_idx += 1;
    });
  }
  else {
    console.log("prop negotationstatus empty");
  }

  return (
    <TreeView
      className={classes.root}
      defaultCollapseIcon={<ExpandMoreIcon />}
      defaultExpanded={['root']}
      defaultExpandIcon={<ChevronRightIcon />}
    >
      {negotiation_contents}
    </TreeView>
  );
}