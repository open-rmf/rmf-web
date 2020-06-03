/**
 * TODO: Show indicator why lift controls are disabled.
 */

import {
  Button,
  Divider,
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelProps,
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
import React, { CSSProperties } from 'react';
import DisableableTypography from './disableable-typography';

export interface LiftItemProps extends Omit<ExpansionPanelProps, 'children'> {
  lift: Readonly<RomiCore.Lift>;
  liftState?: Readonly<RomiCore.LiftState>;
  enableRequest?: boolean;
  onRequest?(lift: RomiCore.Lift, destination: string): void;
}

export const LiftItem = React.forwardRef(function(
  props: LiftItemProps,
  ref: React.Ref<HTMLElement>,
): React.ReactElement {
  const { lift, liftState, enableRequest, onRequest, ...otherProps } = props;
  const theme = useTheme();
  const classes = useStyles();

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
      return (
        <DisableableTypography disabled={!liftState} variant="body1">
          Unknown
        </DisableableTypography>
      );
    }
    return renderList(liftState.available_floors);
  }

  function renderAvailableModes(liftState?: RomiCore.LiftState): JSX.Element {
    if (!liftState) {
      return (
        <DisableableTypography disabled={!liftState} variant="body1">
          Unknown
        </DisableableTypography>
      );
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

  function handleRequestClick(event: React.MouseEvent, lift: RomiCore.Lift): void {
    setLiftRequestMenuState({
      lift: lift,
      anchor: { top: event.clientY, left: event.clientX },
    });
  }

  return (
    <React.Fragment>
      <ExpansionPanel ref={ref} {...otherProps}>
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
            <DisableableTypography disabled={!liftState} variant="body1">
              {liftState ? liftState.destination_floor : 'Unknown'}
            </DisableableTypography>
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Available Floors:</Typography>
            {renderAvailableFloors(liftState)}
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Current Mode:</Typography>
            <DisableableTypography disabled={!liftState} variant="body1">
              {liftState ? liftModeToString(liftState.current_mode) : 'Unknown'}
            </DisableableTypography>
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Available Modes:</Typography>
            {renderAvailableModes(liftState)}
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Door State:</Typography>
            <DisableableTypography disabled={!liftState} variant="body1">
              {liftState ? doorStateToString(liftState.door_state) : 'Unknown'}
            </DisableableTypography>
          </div>
          <Divider />
          <div className={classes.expansionDetailLine}>
            <Typography variant="body1">Motion State:</Typography>
            <DisableableTypography disabled={!liftState} variant="body1">
              {liftState ? motionStateToString(liftState.motion_state) : 'Unknown'}
            </DisableableTypography>
          </div>

          <Button
            variant="outlined"
            style={{ marginTop: theme.spacing(1) }}
            onClick={event => handleRequestClick(event, lift)}
            disabled={!enableRequest}
            fullWidth
          >
            Request
          </Button>
        </ExpansionPanelDetails>
      </ExpansionPanel>
      {liftRequestMenuState && (
        <Popover
          anchorReference="anchorPosition"
          anchorPosition={liftRequestMenuState?.anchor}
          open={Boolean(liftRequestMenuState)}
          onClose={() => setLiftRequestMenuState(null)}
        >
          {liftRequestMenuState &&
            liftState?.available_floors.map(floor => (
              <MenuItem
                key={floor}
                onClick={() => {
                  onRequest && onRequest(lift, floor);
                  setLiftRequestMenuState(null);
                }}
              >
                {floor}
              </MenuItem>
            ))}
        </Popover>
      )}
    </React.Fragment>
  );
});

interface LiftRequestMenuState {
  lift: RomiCore.Lift;
  anchor: PopoverPosition;
}

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
    },
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
