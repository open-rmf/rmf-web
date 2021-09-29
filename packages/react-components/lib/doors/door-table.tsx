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
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import { DoorData, doorModeToString, doorTypeToString } from './utils';

export interface DoorTableProps {
  doors: DoorData[];
  doorStates: Record<string, RmfModels.DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export interface DoorRowProps extends ListChildComponentProps {
  data: DoorTableProps;
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

const DoorRow = React.memo(({ data, index, style }: DoorRowProps) => {
  const classes = useStyles();
  const door = data.doors[index];
  const doorState = data.doorStates[door.door.name];
  const onDoorControlClick = data.onDoorControlClick;

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
    <TableRow arial-label={`${door.door.name}`} component="div" style={style}>
      <TableCell component="div" variant="body">
        {door.door.name}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={
          getOpMode(doorState) === 'Offline' ? classes.doorLabelClosed : classes.doorLabelOpen
        }
      >
        {getOpMode(doorState)}
      </TableCell>
      <TableCell component="div" variant="body">
        {door.level}
      </TableCell>
      <TableCell component="div">{doorTypeToString(door.door.door_type)}</TableCell>
      <TableCell className={doorStatusClass} component="div" variant="body">
        {doorModeToString(doorState)}
      </TableCell>
      <TableCell component="div" variant="body">
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
    <Table stickyHeader size="small" aria-label="door-table" component="div">
      <TableHead component="div">
        <TableRow component="div">
          <TableCell component="div" variant="head">
            Door Name
          </TableCell>
          <TableCell component="div" variant="head">
            Op. Mode
          </TableCell>
          <TableCell component="div" variant="head">
            Level
          </TableCell>
          <TableCell component="div" variant="head">
            Door Type
          </TableCell>
          <TableCell component="div" variant="head">
            Doors State
          </TableCell>
          <TableCell component="div" variant="head">
            Door Control
          </TableCell>
        </TableRow>
      </TableHead>
      <TableBody component="div">
        <FixedSizeList
          itemSize={42}
          itemCount={doors.length}
          height={200}
          width={800}
          itemData={{
            doors,
            doorStates,
            onDoorControlClick,
          }}
        >
          {DoorRow}
        </FixedSizeList>
        {/* {doors.map((door) => (
          <DoorRow
            door={door}
            doorState={doorStates[door.door.name]}
            onDoorControlClick={onDoorControlClick}
            key={door.door.name}
          />
        ))} */}
      </TableBody>
    </Table>
  );
};
