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
import clsx from 'clsx';
import React from 'react';
import { LeafletContext } from 'react-leaflet';
import AutoSizer from 'react-virtualized-auto-sizer';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import { DoorMode as RmfDoorMode } from 'rmf-models';
import { useFixedTableCellStyles } from '../utils';
import { DoorData, doorModeToString, doorTypeToString, onDoorClick } from './utils';

export interface DoorTableProps {
  doors: DoorData[];
  doorStates: Record<string, DoorState>;
  leafletMap?: LeafletContext;
  onDoorControlClick?(event: React.MouseEvent, door: Door, mode: number): void;
}

interface DoorListRendererProps extends ListChildComponentProps {
  data: DoorTableProps;
}

export interface DoorRowProps {
  door: DoorData;
  doorMode: number;
  leafletMap?: LeafletContext;
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
  tableRow: {
    '&:hover': {
      cursor: 'pointer',
      backgroundColor: theme.palette.action.hover,
    },
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

const DoorRow = React.memo(({ door, doorMode, leafletMap, onDoorControlClick }: DoorRowProps) => {
  const classes = useStyles();
  const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStyles();
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
    <TableRow
      arial-label={`${door.door.name}`}
      component="div"
      onClick={() => onDoorClick(door.door, leafletMap)}
      className={classes.tableRow}
    >
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, fixedTableCell)}
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
          fixedTableCell,
        )}
      >
        {getOpMode(doorMode)}
      </TableCell>
      <TableCell component="div" variant="body" className={clsx(classes.tableCell, fixedTableCell)}>
        {door.level}
      </TableCell>
      <TableCell component="div" variant="body" className={clsx(classes.tableCell, fixedTableCell)}>
        {doorTypeToString(door.door.door_type)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(doorStatusClass, classes.tableCell, fixedTableCell)}
      >
        {doorModeToString(doorMode)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, fixedLastTableCell)}
      >
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

const DoorListRenderer = ({ data, index, style }: DoorListRendererProps) => {
  const door = data.doors[index];
  const doorState = data.doorStates[door.door.name];
  const leafletMap = data.leafletMap;

  return (
    <div style={style}>
      <DoorRow
        door={door}
        doorMode={doorState?.current_mode.value}
        onDoorControlClick={data.onDoorControlClick}
        leafletMap={leafletMap}
        key={door.door.name}
      />
    </div>
  );
};

export const DoorTable = ({
  doors,
  doorStates,
  leafletMap,
  onDoorControlClick,
}: DoorTableProps): JSX.Element => {
  const classes = useStyles();
  const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStyles();
  return (
    <AutoSizer disableHeight>
      {({ width }) => {
        return (
          <Table size="small" aria-label="door-table" component="div">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow} style={{ width: width }}>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Door Name
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Op. Mode
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Level
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Door Type
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Doors State
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedLastTableCell)}
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
                  leafletMap,
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
