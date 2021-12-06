import { Button, ButtonGroup, styled, Table, TableBody, TableHead, TableRow } from '@mui/material';
import type { Door, DoorState } from 'api-client';
import clsx from 'clsx';
import React from 'react';
import AutoSizer, { AutoSizerProps } from 'react-virtualized-auto-sizer';
import { DoorMode as RmfDoorMode } from 'rmf-models';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import { DoorData, doorModeToString, doorTypeToString } from './utils';
import { useFixedTableCellStylesClasses, ItemTableCell } from '../utils';

export interface DoorTableProps {
  doors: DoorData[];
  doorStates: Record<string, DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: Door, mode: number): void;
}

interface DoorListRendererProps extends ListChildComponentProps {
  data: DoorTableProps;
}

export interface DoorRowProps {
  door: DoorData;
  doorMode: number;
  onDoorControlClick?(event: React.MouseEvent, door: Door, mode: number): void;
}

const classes = {
  doorLabelOpen: 'door-table-label-open',
  doorLabelClosed: 'door-table-label-closed',
  doorLabelMoving: 'door-table-label-moving',
  tableRow: 'door-table-row',
  tableCell: 'door-table-cell',
};
const StyledAutoSizer = styled((props: AutoSizerProps) => <AutoSizer {...props} />)(
  ({ theme }) => ({
    [`& .${classes.doorLabelOpen}`]: {
      color: theme.palette.success.main,
    },
    [`& .${classes.doorLabelClosed}`]: {
      color: theme.palette.error.main,
    },
    [`& .${classes.doorLabelMoving}`]: {
      color: theme.palette.warning.main,
    },
    [`& .${classes.tableRow}`]: {
      '&:hover': {
        cursor: 'pointer',
        backgroundColor: theme.palette.action.hover,
      },
      display: 'flex',
      flexDirection: 'row',
    },
    [`& .${classes.tableCell}`]: {
      whiteSpace: 'nowrap',
      overflow: 'hidden',
      textOverflow: 'ellipsis',
    },
  }),
);

const getOpMode = (doorMode?: number) => {
  const getState = doorModeToString(doorMode);
  return getState === 'N/A' ? 'Offline' : 'Online';
};

const DoorRow = React.memo(({ door, doorMode, onDoorControlClick }: DoorRowProps) => {
  const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStylesClasses;
  const doorModeLabelClasses = React.useCallback((doorMode?: number): string => {
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
  }, []);
  const doorStatusClass = doorModeLabelClasses(doorMode);

  return (
    <TableRow arial-label={`${door.door.name}`} component="div" className={classes.tableRow}>
      <ItemTableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, fixedTableCell)}
        title={door?.door.name}
      >
        {door.door.name}
      </ItemTableCell>
      <ItemTableCell
        component="div"
        variant="body"
        className={clsx(
          getOpMode(doorMode) === 'Offline' ? classes.doorLabelClosed : classes.doorLabelOpen,
          classes.tableCell,
          fixedTableCell,
        )}
      >
        {getOpMode(doorMode)}
      </ItemTableCell>
      <ItemTableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, fixedTableCell)}
      >
        {door.level}
      </ItemTableCell>
      <ItemTableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, fixedTableCell)}
      >
        {doorTypeToString(door.door.door_type)}
      </ItemTableCell>
      <ItemTableCell
        component="div"
        variant="body"
        className={clsx(doorStatusClass, classes.tableCell, fixedTableCell)}
      >
        {doorModeToString(doorMode)}
      </ItemTableCell>
      <ItemTableCell
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
      </ItemTableCell>
    </TableRow>
  );
});

const DoorListRenderer = ({ data, index, style }: DoorListRendererProps) => {
  const door = data.doors[index];
  const doorState = data.doorStates[door.door.name];

  return (
    <div style={style}>
      <DoorRow
        door={door}
        doorMode={doorState?.current_mode.value}
        onDoorControlClick={data.onDoorControlClick}
        key={door.door.name}
      />
    </div>
  );
};

export const DoorTable = ({
  doors,
  doorStates,
  onDoorControlClick,
}: DoorTableProps): JSX.Element => {
  const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStylesClasses;
  return (
    <StyledAutoSizer disableHeight>
      {({ width }) => {
        return (
          <Table size="small" aria-label="door-table" component="div">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow} style={{ width: width }}>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Door Name
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Op. Mode
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Level
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Door Type
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Doors State
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedLastTableCell)}
                >
                  Door Control
                </ItemTableCell>
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
    </StyledAutoSizer>
  );
};
