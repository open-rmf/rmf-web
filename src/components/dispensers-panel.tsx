import {
  Divider,
  ExpansionPanelDetails,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
  useTheme,
} from '@material-ui/core';
import { CSSProperties } from '@material-ui/core/styles/withStyles';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import SpotlightExpansionPanel, { SpotlightValue } from './spotlight-expansion-panel';

import Select from '@material-ui/core/Select';

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
      return 'BUSY';
    case RomiCore.DispenserState.OFFLINE:
      return 'OFFLINE';
    default:
      return 'UNKNOWN';
  }
}

function RequestQueueLengthToString(dispenserState?: RomiCore.DispenserState): number {
  if (!dispenserState) {
    return -1;
  }
  return dispenserState.request_guid_queue.length;
}

function RequestIdToString(index: number, dispenserState?: RomiCore.DispenserState): string {
  if (!dispenserState) {
    return 'unknown';
  } else if (dispenserState.request_guid_queue.length < index) {
    return 'unknown';
  }
  return dispenserState.request_guid_queue[index];
}

function CurrentRequestToString(dispenserState?: RomiCore.DispenserState): string {
  if (!dispenserState) {
    return 'unknown';
  }
  return RequestIdToString(0, dispenserState);
}

function RequestQueueToString(dispenserState?: RomiCore.DispenserState): string {
  if (!dispenserState) {
    return 'unknown';
  }
  let queue_string = '';
  dispenserState.request_guid_queue.map(id => {
    queue_string += id + ', ';
  });
  return queue_string;
}

function SecondsRemainingToString(dispenserState?: RomiCore.DispenserState): number {
  if (!dispenserState) {
    return -1;
  }
  return dispenserState.seconds_remaining;
}

export interface DispenserPanelProps {
  transport?: Readonly<RomiCore.Transport>;
  dispenserStates: Readonly<Record<string, RomiCore.DispenserState | undefined>>;
  spotlight?: SpotlightValue<string>;
}

export default function DispensersPanel(props: DispenserPanelProps): React.ReactElement {
  const theme = useTheme();
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
      return (<div />);
    } else if (dispenserState.request_guid_queue.length == 0) {
      return (<div />);
    }
    return (
      <Select multiple native>
        {dispenserState.request_guid_queue.map(id => (
          <option key={id} value={id}>{id}</option>
        ))}
      </Select>
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
                {RequestQueueLengthToString(state)}
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
                {SecondsRemainingToString(state)}
              </Typography>
            </div>
          </ExpansionPanelDetails>
        </SpotlightExpansionPanel>
    );
  });

  return <React.Fragment>{dispensers}</React.Fragment>;
}
