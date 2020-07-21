import {
  Button,
  ButtonGroup,
  Divider,
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelProps,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
  useTheme,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

export interface DoorItemProps extends Omit<ExpansionPanelProps, 'children'> {
  door: Readonly<RomiCore.Door>;
  doorState?: Readonly<RomiCore.DoorState>;
  enableControls?: boolean;
  onDoorClick?(door: RomiCore.Door): void;
  onOpenClick?(door: RomiCore.Door): void;
  onCloseClick?(door: RomiCore.Door): void;
}

export const DoorItem = React.forwardRef(function(
  props: DoorItemProps,
  ref: React.Ref<HTMLElement>,
): React.ReactElement {
  const { door, doorState, enableControls, onOpenClick, onCloseClick, ...otherProps } = props;
  const classes = useStyles();
  const theme = useTheme();

  function doorModeLabelClasses(doorState?: RomiCore.DoorState): string {
    if (!doorState) {
      return `${classes.doorLabel} ${classes.unknown}`;
    }
    switch (doorState.current_mode.value) {
      case RomiCore.DoorMode.MODE_OPEN:
        return `${classes.doorLabel} ${classes.doorLabelOpen}`;
      case RomiCore.DoorMode.MODE_CLOSED:
        return `${classes.doorLabel} ${classes.doorLabelClosed}`;
      case RomiCore.DoorMode.MODE_MOVING:
        return `${classes.doorLabel} ${classes.doorLabelMoving}`;
      default:
        return '';
    }
  }

  return (
    <ExpansionPanel
      ref={ref}
      data-component="DoorItem"
      data-name={door.name}
      data-state={doorModeToString(doorState)}
      {...otherProps}
    >
      <ExpansionPanelSummary
        classes={{ content: classes.expansionSummaryContent }}
        expandIcon={<ExpandMoreIcon />}
      >
        <Typography variant="h5" className={classes.hideText}>{door.name}</Typography>
        <Typography data-role="state" className={doorModeLabelClasses(doorState)} variant="button">
          {doorModeToString(doorState)}
        </Typography>
      </ExpansionPanelSummary>
      <ExpansionPanelDetails data-role="details" className={classes.expansionDetail}>
        <div className={classes.expansionDetailLine}>
          <Typography variant="body1">Type:</Typography>
          <Typography variant="body1">{doorTypeToString(door.door_type)}</Typography>
        </div>
        <Divider />
        <div className={classes.expansionDetailLine}>
          <Typography variant="body1">Motion Direction:</Typography>
          <Typography variant="body1">{motionDirectionToString(door.motion_direction)}</Typography>
        </div>
        <Divider />
        <div className={classes.expansionDetailLine}>
          <Typography variant="body1">Motion Range:</Typography>
          <Typography variant="body1">{door.motion_range}</Typography>
        </div>
        <Divider />
        <div className={classes.expansionDetailLine}>
          <Typography variant="body1">Location:</Typography>
          <Typography variant="body1">
            ({door.v1_x.toFixed(3)}, {door.v1_y.toFixed(3)})
          </Typography>
        </div>
        <ButtonGroup style={{ marginTop: theme.spacing(1) }} fullWidth disabled={!enableControls}>
          <Button onClick={() => onCloseClick && onCloseClick(door)}>Close</Button>
          <Button onClick={() => onOpenClick && onOpenClick(door)}>Open</Button>
        </ButtonGroup>
      </ExpansionPanelDetails>
    </ExpansionPanel>
  );
});

export default DoorItem;

const useStyles = makeStyles(theme => ({
  expansionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  expansionDetail: {
    flexFlow: 'column',
    padding: '8px'
  },

  expansionDetailLine: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },

  doorLabel: {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    border: 2,
    padding: 5,
    width: '4rem',
    textAlign: 'center',
    overflow: "hidden",
    textOverflow: "ellipsis",
    whiteSpace: "nowrap",
  },

  doorLabelOpen: {
    borderColor: theme.palette.success.main,
  },

  doorLabelClosed: {
    borderColor: theme.palette.error.main,
  },

  doorLabelMoving: {
    borderColor: theme.palette.warning.main,
  },

  unknown: {
    borderColor: theme.palette.secondary.dark,
  },

  hideText: {
    overflow: "hidden",
    textOverflow: "ellipsis",
    whiteSpace: "nowrap",
    maxWidth: "10rem"
  }
}));

function doorTypeToString(doorType: number): string {
  switch (doorType) {
    case RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING:
      return 'Double Sliding';
    case RomiCore.Door.DOOR_TYPE_DOUBLE_SWING:
      return 'Double Swing';
    case RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE:
      return 'Double Telescope';
    case RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING:
      return 'Sliding';
    case RomiCore.Door.DOOR_TYPE_SINGLE_TELESCOPE:
      return 'Telescope';
    default:
      return `Unknown (${doorType})`;
  }
}

function doorModeToString(doorState?: RomiCore.DoorState): string {
  if (!doorState) {
    return 'UNKNOWN';
  }
  switch (doorState.current_mode.value) {
    case RomiCore.DoorMode.MODE_OPEN:
      return 'OPEN';
    case RomiCore.DoorMode.MODE_CLOSED:
      return 'CLOSED';
    case RomiCore.DoorMode.MODE_MOVING:
      return 'MOVING';
    default:
      return 'UNKNOWN';
  }
}

function motionDirectionToString(motionDirection: number): string {
  switch (motionDirection) {
    case 1:
      return 'Clockwise';
    case -1:
      return 'Anti-Clockwise';
    default:
      return `Unknown (${motionDirection})`;
  }
}
