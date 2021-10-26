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
import React from 'react';
import * as RmfModels from 'rmf-models';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import { DoorData, doorModeToString, doorTypeToString, doorTableCellConfig } from './utils';
import clsx from 'clsx';
import AutoSizer from 'react-virtualized-auto-sizer';

export interface DoorTableProps {
  doors: DoorData[];
  doorStates: Record<string, RmfModels.DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export interface DoorFixListDataProps extends DoorTableProps {
  width: number;
}

export interface DoorRowProps extends ListChildComponentProps {
  data: DoorFixListDataProps;
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

const getOpMode = (doorMode?: number) => {
  const getState = doorModeToString(doorMode);
  return getState === 'N/A' ? 'Offline' : 'Online';
};

const DoorRow = React.memo(({ data, index, style }: DoorRowProps) => {
  const classes = useStyles();
  const door = data.doors[index];
  const doorMode = data.doorStates[door.door.name]?.current_mode.value;
  const width = data.width;
  const onDoorControlClick = data.onDoorControlClick;

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
  -40;
  const doorStatusClass = doorModeLabelClasses(doorMode);

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
        style={{
          minWidth: width * doorTableCellConfig.doorName - 40,
          height: doorTableCellConfig.rowHeight,
        }}
        title={door?.door.name}
      >
        {door.door.name}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(
          getOpMode(doorMode) === 'Offline' ? classes.doorLabelClosed : classes.doorLabelOpen,
          classes.tableCell,
          classes.expandingCell,
        )}
        style={{
          minWidth: width * doorTableCellConfig.doorMode - 40,
          height: doorTableCellConfig.rowHeight,
        }}
      >
        {getOpMode(doorMode)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{
          minWidth: width * doorTableCellConfig.doorLevel - 40,
          height: doorTableCellConfig.rowHeight,
        }}
      >
        {door.level}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{
          minWidth: width * doorTableCellConfig.doorType - 40,
          height: doorTableCellConfig.rowHeight,
        }}
      >
        {doorTypeToString(door.door.door_type)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(doorStatusClass, classes.tableCell, classes.expandingCell)}
        style={{
          minWidth: width * doorTableCellConfig.doorState - 40,
          height: doorTableCellConfig.rowHeight,
        }}
      >
        {doorModeToString(doorMode)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{
          minWidth: width * doorTableCellConfig.doorControl - 32,
          height: doorTableCellConfig.rowHeight,
        }}
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
  const [width, setWidth] = React.useState(0);
  return (
    <Table stickyHeader size="small" aria-label="door-table" component="div">
      <TableHead component="div">
        <TableRow component="div" className={classes.tableRow}>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: width * doorTableCellConfig.doorName - 40,
              height: doorTableCellConfig.rowHeight,
            }}
          >
            Door Name
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: width * doorTableCellConfig.doorMode - 40,
              height: doorTableCellConfig.rowHeight,
            }}
          >
            Op. Mode
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: width * doorTableCellConfig.doorLevel - 40,
              height: doorTableCellConfig.rowHeight,
            }}
          >
            Level
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: width * doorTableCellConfig.doorType - 40,
              height: doorTableCellConfig.rowHeight,
            }}
          >
            Door Type
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: width * doorTableCellConfig.doorState - 40,
              height: doorTableCellConfig.rowHeight,
            }}
          >
            Doors State
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: width * doorTableCellConfig.doorControl - 32,
              height: doorTableCellConfig.rowHeight,
            }}
          >
            Door Control
          </TableCell>
        </TableRow>
      </TableHead>
      <TableBody component="div" className={classes.tableBody}>
        <AutoSizer disableHeight>
          {({ width }) => {
            console.log(width);
            setWidth(width);
            return (
              <FixedSizeList
                itemSize={43}
                itemCount={doors.length}
                height={200}
                width={width}
                itemData={{
                  doors,
                  doorStates,
                  width,
                  onDoorControlClick,
                }}
              >
                {DoorRow}
              </FixedSizeList>
            );
          }}
        </AutoSizer>
      </TableBody>
    </Table>
  );
};
