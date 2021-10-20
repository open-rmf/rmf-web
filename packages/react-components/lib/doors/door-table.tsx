import {
  Button,
  ButtonGroup,
  styled,
  Table,
  TableProps,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
} from '@mui/material';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { DoorData, doorModeToString, doorTypeToString } from './utils';

export interface DoorTableProps {
  doors: DoorData[];
  doorStates: Record<string, RmfModels.DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export interface DoorRowProps {
  door: DoorData;
  doorMode?: number;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

const classes = {
  doorLabelOpen: 'door-table-label-open',
  doorLabelClosed: 'door-table-label-closed',
  doorLabelMoving: 'door-table-label-moving',
};
const DoorTableRoot = styled((props: TableProps) => <Table {...props} />)(({ theme }) => ({
  [`& .${classes.doorLabelOpen}`]: {
    color: theme.palette.success.main,
  },
  [`& .${classes.doorLabelClosed}`]: {
    color: theme.palette.error.main,
  },
  [`& .${classes.doorLabelMoving}`]: {
    color: theme.palette.warning.main,
  },
}));

const getOpMode = (doorMode?: number) => {
  const getState = doorModeToString(doorMode);
  return getState === 'N/A' ? 'Offline' : 'Online';
};

const DoorRow = React.memo(({ door, doorMode, onDoorControlClick }: DoorRowProps) => {
  const doorModeLabelClasses = React.useCallback(
    (doorMode?: number): string => {
      if (doorMode === undefined) {
        return '';
      }
      switch (doorMode) {
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
    <DoorTableRoot stickyHeader size="small" aria-label="door-table">
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
    </DoorTableRoot>
  );
};
