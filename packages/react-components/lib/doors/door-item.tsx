import {
  Accordion,
  AccordionDetails,
  AccordionProps,
  AccordionSummary,
  Button,
  ButtonGroup,
  makeStyles,
  useTheme,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import SimpleInfo from '../simple-info';
import StatusLabel from '../status-label';

const debug = Debug('Doors:DoorItem');

const useStyles = makeStyles((theme) => ({
  accordionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  accordionDetail: {
    flexFlow: 'column',
    padding: theme.spacing(1),
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
    borderColor: '#cccccc',
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

interface DoorInfoProps {
  door: RomiCore.Door;
}

const DoorInfo = (props: DoorInfoProps) => {
  const { door } = props;

  const data = {
    Name: door.name,
    Type: doorTypeToString(door.door_type),
    'Motion Direction': motionDirectionToString(door.motion_direction),
    'Motion Range': door.motion_range,
    Location: `(${door.v1_x.toFixed(3)}, ${door.v1_y.toFixed(3)})`,
  };

  return <SimpleInfo data={data} />;
};

export interface DoorItemProps extends Omit<AccordionProps, 'children'> {
  door: RomiCore.Door;
  doorState?: RomiCore.DoorState;
  onDoorOpen?(door: RomiCore.Door): void;
  onDoorClose?(door: RomiCore.Door): void;
}

export const DoorItem = React.memo(
  React.forwardRef((props: DoorItemProps, ref: React.Ref<HTMLElement>) => {
    const { door, doorState, onDoorOpen, onDoorClose, ...otherProps } = props;
    debug(`render ${door.name}`);

    const classes = useStyles();
    const theme = useTheme();

    const doorModeLabelClasses = (doorState?: RomiCore.DoorState): string => {
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
    };

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
          <StatusLabel
            modalLabelClass={doorModeLabelClasses(doorState)}
            name={door.name}
            modeText={doorModeToString(doorState)}
          />
        </AccordionSummary>
        <AccordionDetails data-role="details" className={classes.accordionDetail}>
          <DoorInfo door={door} />
          <ButtonGroup style={{ marginTop: theme.spacing(1) }} fullWidth>
            <Button disabled={!onDoorOpen} onClick={() => onDoorOpen!(door)}>
              Close
            </Button>
            <Button disabled={!onDoorClose} onClick={() => onDoorClose!(door)}>
              Open
            </Button>
          </ButtonGroup>
        </AccordionDetails>
      </Accordion>
    );
  }),
);

export default DoorItem;
