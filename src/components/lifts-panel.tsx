/**
 * TODO: Show indicator why lift controls are disabled.
 */

import {
  Button,
  Divider,
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelSummary,
  List,
  ListItem,
  makeStyles,
  MenuItem,
  Popover,
  PopoverPosition,
  Typography,
  useTheme,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { CSSProperties, MouseEvent } from 'react';

const useStyles = makeStyles(theme => {
  const liftFloorLabelBase: CSSProperties = {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',

    border: 2,
    padding: 5,
    width: '4rem',
    textAlign: 'center',
  };

  return {
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

    noPadding: {
      padding: 0,
    },

    liftFloorLabelStopped: {
      ...liftFloorLabelBase,
      borderColor: theme.palette.info.main,
    },

    liftFloorLabelMoving: {
      ...liftFloorLabelBase,
      borderColor: theme.palette.warning.main,
    },

    liftFloorLabelUnknown: {
      ...liftFloorLabelBase,
      borderStyle: 'none',
    }
  };
});

function liftModeToString(liftMode: number): string {
  switch (liftMode) {
    case RomiCore.LiftState.MODE_AGV:
      return 'AGV';
    case RomiCore.LiftState.MODE_EMERGENCY:
      return 'Emergency';
    case RomiCore.LiftState.MODE_FIRE:
      return 'Fire';
    case RomiCore.LiftState.MODE_HUMAN:
      return 'Human';
    default:
      return `Unknown (${liftMode})`;
  }
}

function doorStateToString(doorState: number): string {
  switch (doorState) {
    case RomiCore.LiftState.DOOR_OPEN:
      return 'Open';
    case RomiCore.LiftState.DOOR_CLOSED:
      return 'Closed';
    case RomiCore.LiftState.DOOR_MOVING:
      return 'Moving';
    default:
      return `Unknown (${doorState})`;
  }
}

function motionStateToString(motionState: number): string {
  switch (motionState) {
    case RomiCore.LiftState.MOTION_DOWN:
      return 'Down';
    case RomiCore.LiftState.MOTION_STOPPED:
      return 'Stopped';
    case RomiCore.LiftState.MOTION_UP:
      return 'Up';
    default:
      return `Unknown (${motionState})`;
  }
}

interface LiftsPanelProps {
  transport?: Readonly<RomiCore.Transport>;
  lifts: readonly RomiCore.Lift[];
  liftStates: Readonly<Record<string, RomiCore.LiftState | undefined>>;
  onLiftRequest?: (lift: RomiCore.Lift, destination: string) => void;
}

interface LiftRequestMenuState {
  lift: RomiCore.Lift;
  anchor: PopoverPosition;
}

type LiftRequestPublisher = RomiCore.Publisher<RomiCore.LiftRequest>;

export default function LiftsPanel(props: LiftsPanelProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();

  const [liftRequestPub, setLiftRequestPub] = React.useState<LiftRequestPublisher | null>(null);
  const [
    liftRequestMenuState,
    setLiftRequestMenuState,
  ] = React.useState<LiftRequestMenuState | null>(null);

  function renderList(values: string[]): JSX.Element {
    const items = values.map(val => (
      <ListItem key={val} dense className={classes.noPadding}>
        <Typography variant="body1">{val}</Typography>
      </ListItem>
    ));
    return <List>{items}</List>;
  }

  function renderAvailableFloors(liftState?: RomiCore.LiftState): JSX.Element {
    if (!liftState) {
      return <Typography variant="body1">Unknown</Typography>;
    }
    return renderList(liftState.available_floors);
  }

  function renderAvailableModes(liftState?: RomiCore.LiftState): JSX.Element {
    if (!liftState) {
      return <Typography variant="body1">Unknown</Typography>;
    }
    const modes = Array.from(liftState.available_modes.values());
    return renderList(modes.map(liftModeToString));
  }

  function liftFloorLabel(liftState?: RomiCore.LiftState): string {
    if (!liftState) {
      return classes.liftFloorLabelUnknown;
    }
    switch (liftState.motion_state) {
      case RomiCore.LiftState.MOTION_UP:
      case RomiCore.LiftState.MOTION_DOWN:
        return classes.liftFloorLabelMoving;
      default:
        return classes.liftFloorLabelStopped;
    }
  }

  function handleRequestClick(event: MouseEvent, lift: RomiCore.Lift): void {
    setLiftRequestMenuState({
      lift: lift,
      anchor: { top: event.clientY, left: event.clientX },
    });
  }

  function renderRequestMenu(lift: RomiCore.Lift): JSX.Element[] {
    return lift.levels.map(floor => (
      <MenuItem
        key={floor}
        onClick={() => {
          liftRequestPub?.publish({
            lift_name: lift.name,
            destination_floor: floor,
            door_state: RomiCore.LiftRequest.DOOR_OPEN,
            request_time: RomiCore.toRosTime(new Date()),
            request_type: RomiCore.LiftRequest.REQUEST_AGV_MODE,
            session_id: props.transport!.name,
          });
          props.onLiftRequest && props.onLiftRequest(lift, floor);
          setLiftRequestMenuState(null);
        }}
      >
        {floor}
      </MenuItem>
    ));
  }

  React.useEffect(() => {
    setLiftRequestPub(
      props.transport ? props.transport.createPublisher(RomiCore.liftRequests) : null,
    );
  }, [props.transport]);

  const listItems = props.lifts.map(lift => {
    const liftState = props.liftStates[lift.name];

    return (
      <ExpansionPanel key={lift.name}>
        <ExpansionPanelSummary
          classes={{ content: classes.expansionSummaryContent }}
          expandIcon={<ExpandMoreIcon />}
        >
          <Typography variant="h5">{lift.name}</Typography>
          <Typography className={liftFloorLabel(liftState)} variant="button">
            {liftState ? liftState.current_floor : 'Unknown'}
          </Typography>
        </ExpansionPanelSummary>
        <ExpansionPanelDetails className={classes.expansionDetail}>
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Location:</Typography>
            <Typography variant="body1">
              {`(${lift.ref_x.toFixed(3)}, ${lift.ref_y.toFixed(3)})`}
            </Typography>
          </div>
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Destination Floor:</Typography>
            <Typography variant="body1">
              {liftState ? liftState.destination_floor : 'Unknown'}
            </Typography>
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Available Floors:</Typography>
            {renderAvailableFloors(liftState)}
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Current Mode:</Typography>
            <Typography variant="body1">
              {liftState ? liftModeToString(liftState.current_mode) : 'Unknown'}
            </Typography>
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Available Modes:</Typography>
            {renderAvailableModes(liftState)}
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Door State:</Typography>
            <Typography variant="body1">
              {liftState ? doorStateToString(liftState.door_state) : 'Unknown'}
            </Typography>
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Motion State:</Typography>
            <Typography variant="body1">
              {liftState ? motionStateToString(liftState.motion_state) : 'Unknown'}
            </Typography>
          </div>

          <Button
            variant="outlined"
            style={{ marginTop: theme.spacing(1) }}
            onClick={event => handleRequestClick(event, lift)}
            disabled={Boolean(!liftRequestPub)}
            fullWidth
          >
            Request
          </Button>
        </ExpansionPanelDetails>
      </ExpansionPanel>
    );
  });

  return (
    <React.Fragment>
      {listItems}
      <Popover
        anchorReference="anchorPosition"
        anchorPosition={liftRequestMenuState?.anchor}
        open={Boolean(liftRequestMenuState && liftRequestPub)}
        onClose={() => setLiftRequestMenuState(null)}
      >
        {liftRequestMenuState && renderRequestMenu(liftRequestMenuState.lift)}
      </Popover>
    </React.Fragment>
  );
}
