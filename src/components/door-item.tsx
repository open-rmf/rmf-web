import {
  Button,
  ButtonGroup,
  Divider,
  Accordion,
  AccordionDetails,
  AccordionProps,
  AccordionSummary,
  makeStyles,
  Typography,
  useTheme,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { colorPalette } from '../util/css-utils';
import OmniPanelStatusLabels from './omni-panel-status-labels';

const debug = Debug('OmniPanel:DoorItem');

export interface DoorItemProps extends Omit<AccordionProps, 'children'> {
  door: Readonly<RomiCore.Door>;
  doorState?: Readonly<RomiCore.DoorState>;
  enableControls?: boolean;
  onDoorClick?(door: RomiCore.Door): void;
  onOpenClick?(door: RomiCore.Door): void;
  onCloseClick?(door: RomiCore.Door): void;
}

export const DoorItem = React.forwardRef(function (
  props: DoorItemProps,
  ref: React.Ref<HTMLElement>,
): React.ReactElement {
  debug('render');

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
        return `${classes.doorLabel} ${classes.unknown}`;
    }
  }

  return (
    <Accordion
      ref={ref}
      data-component="DoorItem"
      data-name={door.name}
      data-state={doorModeToString(doorState)}
      {...otherProps}
    >
      <AccordionSummary
        classes={{ content: classes.accordionSummaryContent }}
        expandIcon={<ExpandMoreIcon />}
      >
        <OmniPanelStatusLabels
          modalLabelClass={doorModeLabelClasses(doorState)}
          name={door.name}
          modeText={doorModeToString(doorState)}
        />
      </AccordionSummary>
      <AccordionDetails data-role="details" className={classes.accordionDetail}>
        <div className={classes.accordionDetailLine}>
          <Typography variant="body1">Name:</Typography>
          <Typography variant="body1">{door.name}</Typography>
        </div>
        <Divider />
        <div className={classes.accordionDetailLine}>
          <Typography variant="body1">Type:</Typography>
          <Typography variant="body1">{doorTypeToString(door.door_type)}</Typography>
        </div>
        <Divider />
        <div className={classes.accordionDetailLine}>
          <Typography variant="body1">Motion Direction:</Typography>
          <Typography variant="body1">{motionDirectionToString(door.motion_direction)}</Typography>
        </div>
        <Divider />
        <div className={classes.accordionDetailLine}>
          <Typography variant="body1">Motion Range:</Typography>
          <Typography variant="body1">{door.motion_range}</Typography>
        </div>
        <Divider />
        <div className={classes.accordionDetailLine}>
          <Typography variant="body1">Location:</Typography>
          <Typography variant="body1">
            ({door.v1_x.toFixed(3)}, {door.v1_y.toFixed(3)})
          </Typography>
        </div>
        <ButtonGroup
          style={{ marginTop: theme.spacing(1) }}
          fullWidth
          disabled={!enableControls}
          data-name={'door-button-group'}
        >
          <Button onClick={() => onCloseClick && onCloseClick(door)}>Close</Button>
          <Button onClick={() => onOpenClick && onOpenClick(door)}>Open</Button>
        </ButtonGroup>
      </AccordionDetails>
    </Accordion>
  );
});

export default DoorItem;

const useStyles = makeStyles((theme) => ({
  accordionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  accordionDetail: {
    flexFlow: 'column',
    padding: '8px',
  },

  accordionDetailLine: {
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
    borderColor: colorPalette.unknown,
  },
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
    return 'N/A';
  }
  switch (doorState.current_mode.value) {
    case RomiCore.DoorMode.MODE_OPEN:
      return 'OPEN';
    case RomiCore.DoorMode.MODE_CLOSED:
      return 'CLOSED';
    case RomiCore.DoorMode.MODE_MOVING:
      return 'MOVING';
    default:
      return 'N/A';
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
