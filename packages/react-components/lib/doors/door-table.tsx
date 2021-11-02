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
import { tableCellStyle } from '../utils';
import clsx from 'clsx';
import AutoSizer from 'react-virtualized-auto-sizer';

export interface DoorTableProps {
  doors: DoorData[];
  doorStates: Record<string, RmfModels.DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

interface DoorListRendererProps extends ListChildComponentProps {
  data: DoorTableProps;
}

export interface DoorRowProps {
  door: DoorData;
  doorMode: number;
  style: React.CSSProperties;
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
  tableRow: {
    display: 'flex',
    flexDirection: 'row',
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

const DoorRow = React.memo(({ door, doorMode, style, onDoorControlClick }: DoorRowProps) => {
  const classes = useStyles();
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
        style={tableCellStyle('1')}
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
        style={tableCellStyle('1')}
      >
        {getOpMode(doorMode)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={classes.tableCell}
        style={tableCellStyle('1')}
      >
        {door.level}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={classes.tableCell}
        style={tableCellStyle('1')}
      >
        {doorTypeToString(door.door.door_type)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(doorStatusClass, classes.tableCell)}
        style={tableCellStyle('1')}
      >
        {doorModeToString(doorMode)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={classes.tableCell}
        style={tableCellStyle('1.5')}
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

const DoorListRenderer = ({ data, index, style }: DoorListRendererProps) => {
  const door = data.doors[index];
  const doorState = data.doorStates[door.door.name];

  return (
    <DoorRow
      door={door}
      doorMode={doorState?.current_mode.value}
      onDoorControlClick={data.onDoorControlClick}
      style={style}
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
              <TableRow component="div" className={classes.tableRow} style={{ width: width }}>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1')}
                >
                  Door Name
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1')}
                >
                  Op. Mode
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1')}
                >
                  Level
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1')}
                >
                  Door Type
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1')}
                >
                  Doors State
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1.5')}
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
