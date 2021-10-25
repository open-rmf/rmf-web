import {
  Button,
  ButtonGroup,
  makeStyles,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
} from '@material-ui/core';
import type { Door, DoorState } from 'api-client';
import React from 'react';
import { DoorMode as RmfDoorMode } from 'rmf-models';
import { DoorData, doorModeToString, doorTypeToString } from './utils';

export interface DoorTableProps {
  doors: DoorData[];
  doorStates: Record<string, DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: Door, mode: number): void;
}

export interface DoorRowProps {
  door: DoorData;
  doorMode?: number;
  onDoorControlClick?(event: React.MouseEvent, door: Door, mode: number): void;
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

const getOpMode = (doorMode?: number) => {
  const getState = doorModeToString(doorMode);
  return getState === 'N/A' ? 'Offline' : 'Online';
};

const DoorRow = React.memo(({ door, doorMode, onDoorControlClick }: DoorRowProps) => {
  const classes = useStyles();

  const doorModeLabelClasses = React.useCallback(
    (doorMode?: number): string => {
      if (doorMode === undefined) {
        return '';
      }
      switch (doorMode) {
        case RmfDoorMode.MODE_OPEN:
          return `${classes.doorLabelOpen}`;
        case RmfDoorMode.MODE_CLOSED:
          return `${classes.doorLabelClosed}`;
        case RmfDoorMode.MODE_MOVING:
          return `${classes.doorLabelMoving}`;
        default:
          return '';
      }
    },
    [classes],
  );

  const doorStatusClass = doorModeLabelClasses(doorMode);

  return (
    <TableRow arial-label={`${door.door.name}`}>
      <TableCell>{door.door.name}</TableCell>
      <TableCell
        className={
          getOpMode(doorMode) === 'Offline' ? classes.doorLabelClosed : classes.doorLabelOpen
        }
      >
        {getOpMode(doorMode)}
      </TableCell>
      <TableCell>{door.level}</TableCell>
      <TableCell>{doorTypeToString(door.door.door_type)}</TableCell>
      <TableCell className={doorStatusClass}>{doorModeToString(doorMode)}</TableCell>
      <TableCell>
        <ButtonGroup size="small">
          <Button
            aria-label={`${door.door.name}_open`}
            onClick={(ev) =>
              onDoorControlClick && onDoorControlClick(ev, door.door, RmfDoorMode.MODE_OPEN)
            }
          >
            Open
          </Button>
          <Button
            aria-label={`${door.door.name}_close`}
            onClick={(ev) =>
              onDoorControlClick && onDoorControlClick(ev, door.door, RmfDoorMode.MODE_CLOSED)
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
            doorMode={doorStates[door.door.name]?.current_mode.value}
            onDoorControlClick={onDoorControlClick}
            key={door.door.name}
          />
        ))}
      </TableBody>
    </Table>
  );
};
