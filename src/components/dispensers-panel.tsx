import {
  Divider,
  ExpansionPanelDetails,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
} from '@material-ui/core';
import { CSSProperties } from '@material-ui/core/styles/withStyles';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import SpotlightExpansionPanel, { SpotlightValue } from './spotlight-expansion-panel';

import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemText from '@material-ui/core/ListItemText';

import React from 'react';

const useStyles = makeStyles(theme => ({
  expansionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  expansionDetail: {
    flexFlow: 'column',
  },

  expansionDetailLine: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },

  listRoot: {
    position: 'relative',
    overflow: 'auto',
    maxHeight: 100,
  },
}));

const useDispenserModeLabelStyles = makeStyles(theme => {
  const base: CSSProperties = {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    border: 2,
    padding: 5,
    width: '4rem',
    textAlign: 'center',
  };

  return {
    idle: {
      ...base,
      borderColor: theme.palette.warning.main,
    },

    busy: {
      ...base,
      borderColor: theme.palette.success.main,
    },

    offline: {
      ...base,
      borderColor: theme.palette.error.main,
    },
  };
});

function dispenserModeToString(dispenserState?: RomiCore.DispenserState): string {
  if (!dispenserState) {
    return 'UNKNOWN';
  }
  switch (dispenserState.mode) {
    case RomiCore.DispenserState.IDLE:
      return 'IDLE';
    case RomiCore.DispenserState.BUSY:
      return 'ONLINE';
    case RomiCore.DispenserState.OFFLINE:
      return 'OFFLINE';
    default:
      return 'UNKNOWN';
  }
}

function requestQueueLengthToString(dispenserState?: RomiCore.DispenserState): string {
  if (!dispenserState) {
    return 'unknown';
  }
  return String(dispenserState.request_guid_queue.length);
}

function secondsRemainingToString(dispenserState?: RomiCore.DispenserState): string {
  if (!dispenserState) {
    return 'unknown';
  }
  return String(dispenserState.seconds_remaining);
}

export interface DispenserPanelProps {
  transport?: Readonly<RomiCore.Transport>;
  dispenserStates: Readonly<Record<string, RomiCore.DispenserState | undefined>>;
  spotlight?: SpotlightValue<string>;
}

export default function DispensersPanel(props: DispenserPanelProps): React.ReactElement {
  const classes = useStyles();

  const dispenserModeLabelClasses = useDispenserModeLabelStyles();
  const dispenserModeLabelClass = (dispenserState?: RomiCore.DispenserState) => {
    if (!dispenserState) {
      return '';
    }
    switch (dispenserState.mode) {
      case RomiCore.DispenserState.IDLE:
        return dispenserModeLabelClasses.idle;
      case RomiCore.DispenserState.BUSY:
        return dispenserModeLabelClasses.busy;
      case RomiCore.DispenserState.OFFLINE:
        return dispenserModeLabelClasses.offline;
      default:
        return '';
    }
  };

  const dispenserRequestQueueId = (dispenserState?: RomiCore.DispenserState) => {
    if (!dispenserState) {
      return null;
    } else if (dispenserState.request_guid_queue.length == 0) {
      return null;
    }

    return (
      <List className={classes.listRoot} dense={true}>
        {dispenserState.request_guid_queue.map(id => (
          <ListItem>
            <ListItemText primary={id}></ListItemText>
          </ListItem>
        ))}
      </List>
    );
  }

  const dispensers = Object.keys(props.dispenserStates).map( (guid, index) => {
    const state = props.dispenserStates[guid];
    return (
        <SpotlightExpansionPanel 
            key={guid} index={guid}
            spotlight={props.spotlight} 
            TransitionProps={{ unmountOnExit: true }}>
          <ExpansionPanelSummary
            classes={{ content: classes.expansionSummaryContent }}
            expandIcon={<ExpandMoreIcon />}>
            <Typography variant="h5">{guid}</Typography>
            <Typography className={dispenserModeLabelClass(state)} 
                variant="button">
              {dispenserModeToString(state)}
            </Typography>
          </ExpansionPanelSummary>
          <ExpansionPanelDetails className={classes.expansionDetail}>

            <div className={classes.expansionDetailLine}>
              <Typography variant="body1">No. Queued Requests:</Typography>
              <Typography variant="body1">
                {requestQueueLengthToString(state)}
              </Typography>
            </div>
            <Divider />

            <div className={classes.expansionDetailLine}>
              <Typography variant="body1">Request Queue ID:</Typography>
              {dispenserRequestQueueId(state)}
            </div>
            <Divider />

            <div className={classes.expansionDetailLine}>
              <Typography variant="body1">Seconds Remaining:</Typography>
              <Typography variant="body1">
                {secondsRemainingToString(state)}
              </Typography>
            </div>

          </ExpansionPanelDetails>
        </SpotlightExpansionPanel>
    );
  });

  return (
    <React.Fragment>
      {dispensers}
    </React.Fragment>
  );
}
