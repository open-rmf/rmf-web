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
  ListItemText,
} from '@material-ui/core';
import { CSSProperties } from '@material-ui/core/styles/withStyles';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

export interface DispenserItemProps extends ExpansionPanelProps {
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
        return '';
    }
  }

  function dispenserRequestQueueId(): React.ReactElement{
    return (
      <List className={classes.listRoot} dense={true}>
        {dispenserState.request_guid_queue.map(id => (
          <ListItem key={id}>
            <ListItemText primary={id}></ListItemText>
          </ListItem>
        ))}
      </List>
    );
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
        return 'UNKNOWN';
    }
  }

  return (
    <ExpansionPanel ref={ref} {...otherProps}>
      <ExpansionPanelSummary
        classes={{ content: classes.expansionSummaryContent }}
        expandIcon={<ExpandMoreIcon />}>
        <Typography variant="h5">{dispenserState.guid}</Typography>
        <Typography className={dispenserModeLabelClass()} variant='button'>
          {dispenserModeToString()}
        </Typography>
      </ExpansionPanelSummary>
      <ExpansionPanelDetails className={classes.expansionDetail}>
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
    idle: {...base, borderColor: theme.palette.warning.main},
    busy: {...base, borderColor: theme.palette.success.main},
    offline: {...base, borderColor: theme.palette.error.main}
  };
});
