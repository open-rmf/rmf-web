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
  negotiationStatus?: RomiCore.NegotiationStatus;
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

  const getParticipantNameById = (negotiation : RomiCore.NegotiationStatus, id : number) => {
    for (var i = 0; i <negotiation.participants.length; ++i)
    {
      if (negotiation.participants[i] === id)
        return negotiation.participant_names[i];
    }
    return "Unknown";
  };

  const classes = useStyles();
  const renderNegotiation = (negotiation : RomiCore.NegotiationStatus) => {
    let conflict_label = "Conflict #" + negotiation.conflict_version + ", Participants: ";
    for (var i = 0; i < negotiation.participant_names.length; ++i)
    {
      conflict_label += negotiation.participant_names[i];
      if (i !== (negotiation.participant_names.length - 1))
        conflict_label += ", ";
    }

    var table_dom : JSX.Element[] = [];

    // add rendering DOM for each table
    negotiation.tables.forEach(table => {
      // [main guy -> accounting for]
      let label_text = "";
      if (table.sequence.length > 0)
      {
        label_text = "[";
        var last_idx = (table.sequence.length - 1);
        var initiator = table.sequence[last_idx];
        label_text += getParticipantNameById(negotiation, initiator);

        if (table.sequence.length > 1)
        {
          label_text += " -> ";

          for (var i = 0; i < last_idx; ++i)
          {
            label_text += getParticipantNameById(negotiation, table.sequence[i]);
            if (i !== (last_idx - 1))
              label_text += ", ";
          }
        }
        label_text += "]";
      }
      else
        label_text = "[Unknown]";

      // status handling and background
      let style = classes.ongoing;
      let rejected_status = NegotiationStatusTable.STATUS_REJECTED | 
        NegotiationStatusTable.STATUS_FORFEITED |
        NegotiationStatusTable.STATUS_DEFUNCT;
      if (table.status & NegotiationStatusTable.STATUS_FINISHED)
      {
        label_text += "  [FINISHED]";
        style = classes.finished;
      }
      else if (table.status & rejected_status)
      {
        label_text += "  [FAILED]";
        style = classes.rejected;
      }
      else
        label_text += "  [ONGOING]";
      
      var idx = negotiation.conflict_version + table_dom.length;
      table_dom.push(<TreeItem nodeId="{idx}" key={idx} classes={{label: style, selected: style}} label={label_text}/>)
    });
    
    return (
      <TreeItem key={negotiation.conflict_version} nodeId="{negotiation.conflict_version}" label={conflict_label}>
        { table_dom }
        
      </TreeItem>
    );
  };

  let negotiation_contents
  if (props.negotiationStatus) {
    negotiation_contents = renderNegotiation(props.negotiationStatus);
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