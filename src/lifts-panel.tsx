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

    liftFloorLabelStopped: {
      ...liftFloorLabelBase,
      borderColor: theme.palette.info.main,
    },

    liftFloorLabelMoving: {
      ...liftFloorLabelBase,
      borderColor: theme.palette.warning.main,
    },
  };
});

function renderList(values: string[]): JSX.Element {
  const items = values.map(floor => (
    <ListItem key={floor} dense style={{ padding: 0 }}>
      <Typography variant="body1">{floor}</Typography>
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

function renderAvailableModes(liftState?: RomiCore.LiftState): JSX.Element {
  if (!liftState) {
    return <Typography variant="body1">Unknown</Typography>;
  }
  const modes = Array.from(liftState.available_modes.values());
  return renderList(modes.map(liftModeToString));
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
  buildingMap: RomiCore.BuildingMap;
  liftStates: Partial<{ [key: string]: RomiCore.LiftState }>;
  onLiftRequest?: (lift: RomiCore.Lift, destination: string) => void;
}

interface LiftRequestState {
  lift: RomiCore.Lift;
  liftState: RomiCore.LiftState;
  anchor: PopoverPosition;
}

export default function LiftsPanel(props: LiftsPanelProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();
  const [liftRequestMenuState, setLiftRequestMenuState] = React.useState<
    LiftRequestState | undefined
  >(undefined);

  function liftFloorLabel(liftState?: RomiCore.LiftState): string {
    if (!liftState) {
      return classes.liftFloorLabelStopped;
    }
    switch (liftState.motion_state) {
      case RomiCore.LiftState.MOTION_UP:
      case RomiCore.LiftState.MOTION_DOWN:
        return classes.liftFloorLabelMoving;
      default:
        return classes.liftFloorLabelStopped;
    }
  }

  const handleRequestClick = (
    event: MouseEvent,
    lift: RomiCore.Lift,
    liftState: RomiCore.LiftState,
  ) => {
    setLiftRequestMenuState({
      lift: lift,
      liftState: liftState,
      anchor: { top: event.clientY, left: event.clientX },
    });
  };

  const renderRequestMenu = (state: LiftRequestState) => {
    return state.liftState.available_floors.map(floor => (
      <MenuItem
        key={floor}
        onClick={() => {
          props.onLiftRequest && props.onLiftRequest(state.lift, floor);
          setLiftRequestMenuState(undefined);
        }}
      >
        {floor}
      </MenuItem>
    ));
  };

  const listItems = props.buildingMap.lifts.map(lift => {
    const liftState = props.liftStates[lift.name];
    const canRequestLift = liftState ? true : false;

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
              {`(${lift.ref_x}, ${lift.ref_y})`}
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
              {liftState
                ? motionStateToString(liftState.motion_state)
                : 'Unknown'}
            </Typography>
          </div>

          <Button
            variant="outlined"
            style={{ marginTop: theme.spacing(1) }}
            onClick={event => handleRequestClick(event, lift, liftState!)}
            disabled={!canRequestLift}
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
        open={Boolean(liftRequestMenuState)}
        onClose={() => setLiftRequestMenuState(undefined)}
      >
        {liftRequestMenuState && renderRequestMenu(liftRequestMenuState)}
      </Popover>
    </React.Fragment>
  );
}
