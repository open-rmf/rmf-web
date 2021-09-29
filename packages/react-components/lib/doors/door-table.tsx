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
import { DoorData, doorModeToString, doorTypeToString, doorCellConfig } from './utils';
import clsx from 'clsx';

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
  expandingCell: {
    flex: 1,
  },
  tableRow: {
    display: 'flex',
    flexDirection: 'row',
    flexWrap: 'nowrap',
    alignItems: 'center',
    boxSizing: 'border-box',
    minWidth: '100%',
    width: '100%',
  },
  tableCell: {
    display: 'block',
    flexGrow: 0,
    flexShrink: 0,
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
  tableBody: {
    width: '100%',
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
    <TableRow
      arial-label={`${door.door.name}`}
      component="div"
      className={classes.tableRow}
      style={style}
    >
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{ minWidth: doorCellConfig.doorName, height: doorCellConfig.rowHeight }}
      >
        {door.door.name}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(
          getOpMode(doorState) === 'Offline' ? classes.doorLabelClosed : classes.doorLabelOpen,
          classes.tableCell,
          classes.expandingCell,
        )}
        style={{ minWidth: doorCellConfig.doorMode, height: doorCellConfig.rowHeight }}
      >
        {getOpMode(doorState)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{ minWidth: doorCellConfig.doorLevel, height: doorCellConfig.rowHeight }}
      >
        {door.level}
      </TableCell>
      <TableCell
        component="div"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{ minWidth: doorCellConfig.doorType, height: doorCellConfig.rowHeight }}
      >
        {doorTypeToString(door.door.door_type)}
      </TableCell>
      <TableCell
        className={clsx(doorStatusClass, classes.tableCell, classes.expandingCell)}
        component="div"
        variant="body"
        style={{ minWidth: doorCellConfig.doorState, height: doorCellConfig.rowHeight }}
      >
        {doorModeToString(doorState)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{ minWidth: doorCellConfig.doorControl, height: doorCellConfig.rowHeight }}
      >
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
  const classes = useStyles();
  return (
    <Table stickyHeader size="small" aria-label="door-table" component="div">
      <TableHead component="div">
        <TableRow component="div" className={classes.tableRow}>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{ minWidth: doorCellConfig.doorName, height: doorCellConfig.rowHeight }}
          >
            Door Name
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{ minWidth: doorCellConfig.doorMode, height: doorCellConfig.rowHeight }}
          >
            Op. Mode
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{ minWidth: doorCellConfig.doorLevel, height: doorCellConfig.rowHeight }}
          >
            Level
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{ minWidth: doorCellConfig.doorType, height: doorCellConfig.rowHeight }}
          >
            Door Type
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{ minWidth: doorCellConfig.doorState, height: doorCellConfig.rowHeight }}
          >
            Doors State
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{ minWidth: doorCellConfig.doorControl, height: doorCellConfig.rowHeight }}
          >
            Door Control
          </TableCell>
        </TableRow>
      </TableHead>
      <TableBody component="div" className={classes.tableBody}>
        <FixedSizeList
          itemSize={43}
          itemCount={doors.length}
          height={200}
          width={760}
          itemData={{
            doors,
            doorStates,
            onDoorControlClick,
          }}
        >
          {DoorRow}
        </FixedSizeList>
      </TableBody>
    </Table>
  );
};
