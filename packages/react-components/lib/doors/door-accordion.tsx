import { makeStyles } from '@material-ui/core';
import Accordion, { AccordionProps } from '@material-ui/core/Accordion';
import Button from '@material-ui/core/Button';
import ButtonGroup from '@material-ui/core/ButtonGroup';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import ItemAccordionDetails from '../item-accordion-details';
import ItemAccordionSummary from '../item-accordion-summary';
import { SimpleInfo, SimpleInfoData } from '../simple-info';

const debug = Debug('Doors:DoorAccordion');

const useStyles = makeStyles((theme) => ({
  controlButtonGroup: {
    marginTop: theme.spacing(1),
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
      return 'Single Sliding';
    case RomiCore.Door.DOOR_TYPE_SINGLE_SWING:
      return 'Single Swing';
    case RomiCore.Door.DOOR_TYPE_SINGLE_TELESCOPE:
      return 'Single Telescope';
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

  const data = [
    { name: 'Name', value: door.name },
    { name: 'Type', value: doorTypeToString(door.door_type) },
    { name: 'Motion Direction', value: motionDirectionToString(door.motion_direction) },
    { name: 'Motion Range', value: door.motion_range.toFixed(3) },
    { name: 'Location', value: `(${door.v1_x.toFixed(3)}, ${door.v1_y.toFixed(3)})` },
  ] as SimpleInfoData[];

  return <SimpleInfo data={data} />;
};

export interface DoorAccordionProps extends Omit<AccordionProps, 'children'> {
  door: RomiCore.Door;
  doorState?: RomiCore.DoorState;
  onDoorControlClick?(event: React.MouseEvent, door: RomiCore.Door, mode: number): void;
}

export const DoorAccordion = React.forwardRef(
  (props: DoorAccordionProps, ref: React.Ref<HTMLElement>) => {
    const { door, doorState, onDoorControlClick, ...otherProps } = props;
    debug(`render ${door.name}`);
    const classes = useStyles();

    const doorModeLabelClasses = React.useCallback(
      (doorState?: RomiCore.DoorState): string | null => {
        if (!doorState) {
          return null;
        }
        switch (doorState.current_mode.value) {
          case RomiCore.DoorMode.MODE_OPEN:
            return `${classes.doorLabelOpen}`;
          case RomiCore.DoorMode.MODE_CLOSED:
            return `${classes.doorLabelClosed}`;
          case RomiCore.DoorMode.MODE_MOVING:
            return `${classes.doorLabelMoving}`;
          default:
            return null;
        }
      },
      [classes],
    );

    const doorStatusClass = doorModeLabelClasses(doorState);

    return (
      <Accordion ref={ref} {...otherProps}>
        <ItemAccordionSummary
          title={door.name}
          statusProps={{
            className: doorStatusClass ? doorStatusClass : undefined,
            text: doorModeToString(doorState),
            variant: doorState ? 'normal' : 'unknown',
          }}
        />
        <ItemAccordionDetails>
          <DoorInfo door={door} />
          {onDoorControlClick && (
            <ButtonGroup className={classes.controlButtonGroup} fullWidth>
              <Button onClick={(ev) => onDoorControlClick(ev, door, RomiCore.DoorMode.MODE_CLOSED)}>
                Close
              </Button>
              <Button onClick={(ev) => onDoorControlClick(ev, door, RomiCore.DoorMode.MODE_OPEN)}>
                Open
              </Button>
            </ButtonGroup>
          )}
        </ItemAccordionDetails>
      </Accordion>
    );
  },
);

export default DoorAccordion;
