import React from 'react';
import * as RmfModels from 'rmf-models';
import {
  makeStyles,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  ButtonGroup,
  Button,
} from '@material-ui/core';

import { DoorData, doorModeToString, doorTypeToString } from './utils';

export interface DoorTableProps {
  doors: DoorData[];
  doorStates: Record<string, RmfModels.DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export interface DoorRowProps {
  door: DoorData;
  doorState: RmfModels.DoorState;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

const useStyles = makeStyles((theme) => ({
  doorLabelOpen: {
    color: theme.palette.success.main,
  },
  doorLabelClosed: {
    color: theme.palette.error.main,
  },
  doorLabelMoving: {
    color: theme.palette.warning.main,
  },
}));

const getOpMode = (doorState: RmfModels.DoorState) => {
  const getState = doorModeToString(doorState);
  return getState === 'N/A' ? 'Offline' : 'Online';
};

const DoorRow = React.memo(({ door, doorState, onDoorControlClick }: DoorRowProps) => {
  const classes = useStyles();

  const doorModeLabelClasses = React.useCallback(
    (doorState?: RmfModels.DoorState): string => {
      if (!doorState) {
        return '';
      }
      switch (doorState.current_mode.value) {
        case RmfModels.DoorMode.MODE_OPEN:
          return `${classes.doorLabelOpen}`;
        case RmfModels.DoorMode.MODE_CLOSED:
          return `${classes.doorLabelClosed}`;
        case RmfModels.DoorMode.MODE_MOVING:
          return `${classes.doorLabelMoving}`;
        default:
          return '';
      }
    },
    [classes],
  );

  const doorStatusClass = doorModeLabelClasses(doorState);

  return (
    <TableRow arial-label={`${door.door.name}`}>
      <TableCell>{door.door.name}</TableCell>
      <TableCell
        className={
          getOpMode(doorState) === 'Offline' ? classes.doorLabelClosed : classes.doorLabelOpen
        }
      >
        {getOpMode(doorState)}
      </TableCell>
      <TableCell>{door.level}</TableCell>
      <TableCell>{doorTypeToString(door.door.door_type)}</TableCell>
      <TableCell className={doorStatusClass}>{doorModeToString(doorState)}</TableCell>
      <TableCell>
        <ButtonGroup size="small">
          <Button
            aria-label={`${door.door.name}_open`}
            onClick={(ev) =>
              onDoorControlClick && onDoorControlClick(ev, door.door, RmfModels.DoorMode.MODE_OPEN)
            }
          >
            Open
          </Button>
          <Button
            aria-label={`${door.door.name}_close`}
            onClick={(ev) =>
              onDoorControlClick &&
              onDoorControlClick(ev, door.door, RmfModels.DoorMode.MODE_CLOSED)
            }
          >
            Close
          </Button>
        </ButtonGroup>
      </TableCell>
    </TableRow>
  );
});

export const DoorTable = ({
  doors,
  doorStates,
  onDoorControlClick,
}: DoorTableProps): JSX.Element => {
  return (
    <Table stickyHeader size="small" aria-label="door-table">
      <TableHead>
        <TableRow>
          <TableCell>Door Name</TableCell>
          <TableCell>Op. Mode</TableCell>
          <TableCell>Level</TableCell>
          <TableCell>Door Type</TableCell>
          <TableCell>Doors State</TableCell>
          <TableCell>Door Control</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {doors.map((door) => (
          <DoorRow
            door={door}
            doorState={doorStates[door.door.name]}
            onDoorControlClick={onDoorControlClick}
            key={door.door.name}
          />
        ))}
      </TableBody>
    </Table>
  );
};
