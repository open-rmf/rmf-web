import {
  Divider,
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelProps,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
  List,
  ListItem,
} from '@material-ui/core';
import { CSSProperties } from '@material-ui/core/styles/withStyles';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

import DisableableTypography from './disableable-typography';
import OmniPanelStatusLabels from './omniPanelStatusLabels';

export interface DispenserItemProps extends Omit<ExpansionPanelProps, 'children'> {
  dispenserState: Readonly<RomiCore.DispenserState>;
}

export const DispenserItem = React.forwardRef(function(
    props: DispenserItemProps,
    ref: React.Ref<HTMLElement>,
): React.ReactElement {
  const { dispenserState, ...otherProps } = props;
  const classes = useStyles();
  const dispenserModeLabelClasses = useDispenserModeLabelStyles();

  function dispenserModeLabelClass(): string {
    switch (dispenserState.mode) {
      case RomiCore.DispenserState.IDLE:
        return `${classes.dispenserLabel} ${dispenserModeLabelClasses.idle}`;
      case RomiCore.DispenserState.BUSY:
        return `${classes.dispenserLabel} ${dispenserModeLabelClasses.busy}`;
      case RomiCore.DispenserState.OFFLINE:
        return `${classes.dispenserLabel} ${dispenserModeLabelClasses.offline}`;
      default:
        return `${classes.dispenserLabel} ${dispenserModeLabelClasses.unknown}`;
    }
  }

  function dispenserRequestQueueId(): React.ReactElement {
    if (dispenserState.request_guid_queue.length === 0) {
      return (
        <DisableableTypography disabled={true} variant="body1">
          Unknown
        </DisableableTypography>
      )
    } else {
    return (
        <List className={classes.listRoot} dense={true}>
          {dispenserState.request_guid_queue.map(id => (
            <ListItem key={id} className={classes.listItem}>
              <Typography variant="body1">{id}</Typography>
            </ListItem>
          ))}
        </List>
      );
    }
  }

  function dispenserModeToString(): string {
    switch (dispenserState.mode) {
      case RomiCore.DispenserState.IDLE:
        return 'IDLE';
      case RomiCore.DispenserState.BUSY:
        return 'ONLINE';
      case RomiCore.DispenserState.OFFLINE:
        return 'OFFLINE';
      default:
        return 'N/A';
    }
  }

  return (
    <ExpansionPanel ref={ref} {...otherProps}>
      <ExpansionPanelSummary
        classes={{ content: classes.expansionSummaryContent }}
        expandIcon={<ExpandMoreIcon />}>
          <OmniPanelStatusLabels
            hideTextStyle={classes.hideText}
            modalLabelClass={dispenserModeLabelClass()}
            name={dispenserState.guid}
            modeText={dispenserModeToString()}
          />
      </ExpansionPanelSummary>
      <ExpansionPanelDetails className={classes.expansionDetail}>
        <div className={classes.expansionDetailLine}>
          <Typography variant="body1">Name:</Typography>
          <Typography variant="body1">{dispenserState.guid}</Typography>
        </div>
        <Divider />
        <div className={classes.expansionDetailLine}>
          <Typography variant="body1">No. Queued Requests:</Typography>
          <Typography variant="body1">
            {String(dispenserState.request_guid_queue.length)}
          </Typography>
        </div>
        <Divider />
        <div className={classes.expansionDetailLine}>
          <Typography variant="body1">Request Queue ID:</Typography>
          {dispenserRequestQueueId()}
        </div>
        <Divider />
        <div className={classes.expansionDetailLine}>
          <Typography variant="body1">Seconds Remaining:</Typography>
          <Typography variant="body1">
            {String(dispenserState.seconds_remaining)}
          </Typography>
        </div>
      </ExpansionPanelDetails>
    </ExpansionPanel>
  );
});

export default DispenserItem;

const useStyles = makeStyles(theme => ({
  expansionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  expansionDetail: {
    flexFlow: 'column',
    padding: '8px',
  },

  expansionDetailLine: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },

  dispenserLabel: {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    border: 2,
    padding: 5,
    width: '4rem',
    textAlign: 'center',
  },

  listRoot: {
    position: 'relative',
    overflow: 'auto',
    maxHeight: 100,
    padding: 0
  },

  listItem: {
    paddingTop: 0
  },

  hideText: {
    overflow: "hidden",
    textOverflow: "ellipsis",
    whiteSpace: "nowrap",
    maxWidth: "10rem",
  }
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
    idle: {...base, borderColor: theme.palette.warning.main},
    busy: {...base, borderColor: theme.palette.success.main},
    offline: {...base, borderColor: theme.palette.error.main},
    unknown: {...base, borderColor: '#cccccc'}
  };
});
