import { Button, ButtonGroup, styled, Table, TableBody, TableHead, TableRow } from '@mui/material';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import { DoorData, doorModeToString, doorTypeToString } from './utils';
import { useFixedTableCellStylesClasses, StyledItemTableCell } from '../utils';
import clsx from 'clsx';
import AutoSizer, { AutoSizerProps } from 'react-virtualized-auto-sizer';

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
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
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
    <TableRow arial-label={`${door.door.name}`} component="div" className={classes.tableRow}>
      <StyledItemTableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, fixedTableCell)}
        title={door?.door.name}
      >
        {door.door.name}
      </StyledItemTableCell>
      <StyledItemTableCell
        component="div"
        variant="body"
        className={clsx(
          getOpMode(doorMode) === 'Offline' ? classes.doorLabelClosed : classes.doorLabelOpen,
          classes.tableCell,
          fixedTableCell,
        )}
      >
        {getOpMode(doorMode)}
      </StyledItemTableCell>
      <StyledItemTableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, fixedTableCell)}
      >
        {door.level}
      </StyledItemTableCell>
      <StyledItemTableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, fixedTableCell)}
      >
        {doorTypeToString(door.door.door_type)}
      </StyledItemTableCell>
      <StyledItemTableCell
        component="div"
        variant="body"
        className={clsx(doorStatusClass, classes.tableCell, fixedTableCell)}
      >
        {doorModeToString(doorMode)}
      </StyledItemTableCell>
      <StyledItemTableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, fixedLastTableCell)}
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
      </StyledItemTableCell>
    </TableRow>
  );
});

const DoorListRenderer = ({ data, index }: DoorListRendererProps) => {
  const door = data.doors[index];
  const doorState = data.doorStates[door.door.name];

  return (
    <DoorRow
      door={door}
      doorMode={doorState?.current_mode.value}
      onDoorControlClick={data.onDoorControlClick}
    />
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
          <Table stickyHeader size="small" aria-label="door-table" component="div">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow} style={{ width: width }}>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Door Name
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Op. Mode
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Level
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Door Type
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Doors State
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedLastTableCell)}
                >
                  Door Control
                </StyledItemTableCell>
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
