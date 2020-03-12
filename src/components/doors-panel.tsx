/**
 * TODO: Show indicator why door controls are disabled.
 */

import {
  Button,
  ButtonGroup,
  Divider,
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
  useTheme,
} from '@material-ui/core';
import { CSSProperties } from '@material-ui/core/styles/withStyles';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

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

const useDoorModeLabelStyles = makeStyles(theme => {
  const base: CSSProperties = {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    border: 2,
    padding: 5,
    width: '4rem',
    textAlign: 'center',
  };

  return {
    open: {
      ...base,
      borderColor: theme.palette.success.main,
    },

    closed: {
      ...base,
      borderColor: theme.palette.error.main,
    },

    moving: {
      ...base,
      borderColor: theme.palette.warning.main,
    },
  };
});

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

interface DoorsPanelProps {
  transport?: Readonly<RomiCore.Transport>;
  doors: readonly RomiCore.Door[];
  doorStates: Readonly<Record<string, RomiCore.DoorState | undefined>>;
  onOpenClick?: (door: RomiCore.Door) => void;
  onCloseClick?: (door: RomiCore.Door) => void;
  spotlight?: string;
}

type DoorRequestPublisher = RomiCore.Publisher<RomiCore.DoorRequest>;

export default function DoorsPanel(props: DoorsPanelProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();

  const [doorRequestPub, setDoorRequestPub] = React.useState<DoorRequestPublisher | null>(null);

  React.useEffect(() => {
    setDoorRequestPub(
      props.transport ? props.transport.createPublisher(RomiCore.doorRequests) : null,
    );
  }, [props.transport]);

  const doorModeLabelClasses = useDoorModeLabelStyles();
  const doorModelLabelClass = (doorState?: RomiCore.DoorState) => {
    if (!doorState) {
      return '';
    }
    switch (doorState.current_mode.value) {
      case RomiCore.DoorMode.MODE_OPEN:
        return doorModeLabelClasses.open;
      case RomiCore.DoorMode.MODE_CLOSED:
        return doorModeLabelClasses.closed;
      case RomiCore.DoorMode.MODE_MOVING:
        return doorModeLabelClasses.moving;
      default:
        return '';
    }
  };

  function requestDoor(door: RomiCore.Door, mode: number): void {
    doorRequestPub?.publish({
      door_name: door.name,
      requested_mode: { value: mode },
      requester_id: props.transport!.name,
      request_time: RomiCore.toRosTime(new Date()),
    });
  }

  function handleOpenClick(door: RomiCore.Door): void {
    requestDoor(door, RomiCore.DoorMode.MODE_OPEN);
    props.onOpenClick && props.onOpenClick(door);
  }

  function handleCloseClick(door: RomiCore.Door): void {
    requestDoor(door, RomiCore.DoorMode.MODE_CLOSED);
    props.onCloseClick && props.onCloseClick(door);
  }

  const listItems = props.doors.map(door => {
    const doorState = props.doorStates[door.name];
    return (
      <ExpansionPanel key={door.name} defaultExpanded={props.spotlight === door.name}>
        <ExpansionPanelSummary
          classes={{ content: classes.expansionSummaryContent }}
          expandIcon={<ExpandMoreIcon />}
        >
          <Typography variant="h5">{door.name}</Typography>
          <Typography className={doorModelLabelClass(doorState)} variant="button">
            {doorModeToString(doorState)}
          </Typography>
        </ExpansionPanelSummary>
        <ExpansionPanelDetails className={classes.expansionDetail}>
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Type:</Typography>
            <Typography variant="body1">{doorTypeToString(door.door_type)}</Typography>
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Motion Direction:</Typography>
            <Typography variant="body1">
              {motionDirectionToString(door.motion_direction)}
            </Typography>
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
          <ButtonGroup
            style={{ marginTop: theme.spacing(1) }}
            fullWidth
            disabled={Boolean(!doorRequestPub)}
          >
            <Button onClick={() => handleCloseClick(door)}>Close</Button>
            <Button onClick={() => handleOpenClick(door)}>Open</Button>
          </ButtonGroup>
        </ExpansionPanelDetails>
      </ExpansionPanel>
    );
  });

  return <React.Fragment>{listItems}</React.Fragment>;
}
