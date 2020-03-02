import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import {
  ExpansionPanel,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
  ExpansionPanelDetails,
  Divider,
  List,
  ListItem,
  Button,
  useTheme,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';

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
  },

  liftFloorLabel: {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    borderColor: theme.palette.info.main,
    border: 2,
    padding: 5,
    width: '5em',
    textAlign: 'center',
  },
}));

function renderList(values: string[]): JSX.Element {
  const items = values.map(floor => (
    <ListItem key={floor} dense >
      <Typography variant="body1">{floor}</Typography>
    </ListItem>
  ));
  return (
    <List>
      {items}
    </List>
  );
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
  };
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
  };
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
  };
}

interface LiftsPanelProps {
  buildingMap: RomiCore.BuildingMap;
  liftStates: { [key: string]: RomiCore.LiftState };
}

export default function LiftsPanel(props: LiftsPanelProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();

  const listItems = props.buildingMap.lifts.map(lift => {
    const liftState = props.liftStates[lift.name];
    return (
      <ExpansionPanel key={lift.name}>
        <ExpansionPanelSummary
          classes={{ content: classes.expansionSummaryContent }}
          expandIcon={<ExpandMoreIcon />}
        >
          <Typography variant="h5">{lift.name}</Typography>
          <div className={classes.liftFloorLabel}>
            <Typography variant="button">
              {liftState ? liftState.current_floor : 'Unknown'}
            </Typography>
          </div>
        </ExpansionPanelSummary>
        <ExpansionPanelDetails className={classes.expansionDetail}>
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Available Floors:</Typography>
            {renderAvailableFloors(liftState)}
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Available Modes:</Typography>
            {renderAvailableModes(liftState)}
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Current Floor:</Typography>
            <Typography variant="body1">
              {liftState ? liftState.current_floor : 'Unknown'}
            </Typography>
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
            <Typography variant="body1">Destination Floor:</Typography>
            <Typography variant="body1">
              {liftState ? liftState.destination_floor : 'Unknown'}
            </Typography>
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

          <Button variant="outlined" style={{marginTop: theme.spacing(1)}} fullWidth>
            Request
          </Button>
        </ExpansionPanelDetails>
      </ExpansionPanel>
    );
  });

  return <React.Fragment>{listItems}</React.Fragment>;
}
