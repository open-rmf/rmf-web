import { makeStyles } from '@material-ui/core';
import Accordion, { AccordionProps } from '@material-ui/core/Accordion';
import Button from '@material-ui/core/Button';
import ButtonGroup from '@material-ui/core/ButtonGroup';
import Debug from 'debug';
import React from 'react';
import * as RmfModels from 'rmf-models';
import ItemAccordionDetails from '../item-accordion-details';
import ItemAccordionSummary from '../item-accordion-summary';
import { SimpleInfo, SimpleInfoData } from '../simple-info';
import { Map as LMap } from 'react-leaflet';

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
    case RmfModels.Door.DOOR_TYPE_DOUBLE_SLIDING:
      return 'Double Sliding';
    case RmfModels.Door.DOOR_TYPE_DOUBLE_SWING:
      return 'Double Swing';
    case RmfModels.Door.DOOR_TYPE_DOUBLE_TELESCOPE:
      return 'Double Telescope';
    case RmfModels.Door.DOOR_TYPE_SINGLE_SLIDING:
      return 'Single Sliding';
    case RmfModels.Door.DOOR_TYPE_SINGLE_SWING:
      return 'Single Swing';
    case RmfModels.Door.DOOR_TYPE_SINGLE_TELESCOPE:
      return 'Single Telescope';
    default:
      return `Unknown (${doorType})`;
  }
}

function doorModeToString(doorState?: RmfModels.DoorState): string {
  if (!doorState) {
    return 'N/A';
  }
  switch (doorState.current_mode.value) {
    case RmfModels.DoorMode.MODE_OPEN:
      return 'OPEN';
    case RmfModels.DoorMode.MODE_CLOSED:
      return 'CLOSED';
    case RmfModels.DoorMode.MODE_MOVING:
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

// door centers assume a zoom level of 4
const doorPosMap: { [key: string]: [number, number] } = {
  door1: [-25, 98],
  door2: [-40, 91.2],
  door3: [-41.1, 81.5],
  door4: [-53.4, 76.6],
  door5: [-43, 99],
  door6: [-41.2, 101.5],
  door7: [-39.8, 94.4],
  door8: [-45.7, 158.2],
  door9: [-46, 124],
  door10: [-97.7, 237.3],
  door11: [-74.1, 172],
};

interface DoorInfoProps {
  door: RmfModels.Door;
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

  return <SimpleInfo infoData={data} />;
};

export interface DoorAccordionProps extends Omit<AccordionProps, 'children'> {
  door: RmfModels.Door;
  doorState?: RmfModels.DoorState;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
  mapRef?: React.RefObject<LMap>;
}

export const DoorAccordion = React.forwardRef(
  (props: DoorAccordionProps, ref: React.Ref<HTMLElement>) => {
    const { door, doorState, onDoorControlClick, mapRef, ...otherProps } = props;
    debug(`render ${door.name}`);
    const classes = useStyles();

    function onAccordianClick(door: RmfModels.Door, mapRef?: React.RefObject<LMap>) {
      mapRef?.current?.leafletElement.setView(doorPosMap[door.name], 5);
    }

    const doorModeLabelClasses = React.useCallback(
      (doorState?: RmfModels.DoorState): string | null => {
        if (!doorState) {
          return null;
        }
        switch (doorState.current_mode.value) {
          case RmfModels.DoorMode.MODE_OPEN:
            return `${classes.doorLabelOpen}`;
          case RmfModels.DoorMode.MODE_CLOSED:
            return `${classes.doorLabelClosed}`;
          case RmfModels.DoorMode.MODE_MOVING:
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
          onAccordianClick={() => onAccordianClick(door, mapRef)}
        />
        <ItemAccordionDetails>
          <DoorInfo door={door} />
          {onDoorControlClick && (
            <ButtonGroup className={classes.controlButtonGroup} fullWidth>
              <Button
                onClick={(ev) => onDoorControlClick(ev, door, RmfModels.DoorMode.MODE_CLOSED)}
              >
                Close
              </Button>
              <Button onClick={(ev) => onDoorControlClick(ev, door, RmfModels.DoorMode.MODE_OPEN)}>
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
