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
import { DoorData, doorModeToString, doorTypeToString } from './utils';
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

export interface DoorListRendererProps extends ListChildComponentProps {
  data: DoorFixListDataProps;
}

export interface DoorRowProps {
  door: DoorData;
  doorState: RmfModels.DoorState;
  style: React.CSSProperties;
  width: number;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

// table cell has padding of 16px left and 24px right respectively
// need to deduct 40px away from actual width
const doorTableCellConfig = {
  // column width in percent of row width
  doorName: 0.187,
  doorMode: 0.141,
  doorLevel: 0.129,
  doorType: 0.176,
  doorState: 0.16,
  // last column deduct 32px
  doorControl: 0.207,
  // row height in pixels
  rowHeight: 31,
};

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
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
}));

const getOpMode = (doorMode?: number) => {
  const getState = doorModeToString(doorMode);
  return getState === 'N/A' ? 'Offline' : 'Online';
};

const DoorRow = React.memo(
  ({ door, doorState, style, width, onDoorControlClick }: DoorRowProps) => {
    const classes = useStyles();
    const doorMode = doorState?.current_mode.value;

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
      <TableRow
        arial-label={`${door.door.name}`}
        component="div"
        className={classes.tableRow}
        style={style}
      >
        <TableCell
          component="div"
          variant="body"
          className={classes.tableCell}
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
          className={classes.tableCell}
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
          className={classes.tableCell}
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
          className={clsx(doorStatusClass, classes.tableCell)}
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
          className={classes.tableCell}
          style={{
            minWidth: width * doorTableCellConfig.doorControl - 32,
            height: doorTableCellConfig.rowHeight,
          }}
        >
          <ButtonGroup size="small">
            <Button
              aria-label={`${door.door.name}_open`}
              onClick={(ev) =>
                onDoorControlClick &&
                onDoorControlClick(ev, door.door, RmfModels.DoorMode.MODE_OPEN)
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
  },
);

const DoorListRenderer = ({ data, index, style }: DoorListRendererProps) => {
  const door = data.doors[index];
  const doorState = data.doorStates[door.door.name];

  return (
    <DoorRow
      door={door}
      doorState={doorState}
      onDoorControlClick={data.onDoorControlClick}
      style={style}
      width={data.width}
    />
  );
};

export const DoorTable = ({
  doors,
  doorStates,
  onDoorControlClick,
}: DoorTableProps): JSX.Element => {
  const classes = useStyles();
  return (
    <AutoSizer disableHeight>
      {({ width }) => {
        return (
          <Table stickyHeader size="small" aria-label="door-table" component="div">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow}>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
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
                  className={classes.tableCell}
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
                  className={classes.tableCell}
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
                  className={classes.tableCell}
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
                  className={classes.tableCell}
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
                  className={classes.tableCell}
                  style={{
                    minWidth: width * doorTableCellConfig.doorControl - 32,
                    height: doorTableCellConfig.rowHeight,
                  }}
                >
                  Door Control
                </TableCell>
              </TableRow>
            </TableHead>
            <TableBody component="div">
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
                {DoorListRenderer}
              </FixedSizeList>
            </TableBody>
          </Table>
        );
      }}
    </AutoSizer>
  );
};
