import { Button, Table, TableBody, TableHead, TableRow, styled } from '@mui/material';
import type { Lift, LiftState } from 'api-client';
import clsx from 'clsx';
import React from 'react';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import { DoorMode as RmfDoorMode } from 'rmf-models';
import { useFixedTableCellStylesClasses, ItemTableCell } from '../utils';
import LiftRequestFormDialog from './lift-request-form-dialog';
import { doorStateToString, liftModeToString, requestDoorModes, requestModes } from './lift-utils';
import AutoSizer, { AutoSizerProps } from 'react-virtualized-auto-sizer';

const classes = {
  doorLabelOpen: 'lift-table-doorlabelopen',
  doorLabelClosed: 'lift-table-doorlabelclosed',
  doorLabelMoving: 'lift-table-doorlabelmoving',
  tableRow: 'lift-table-row',
  tableCell: 'lift-table-cell',
};
const StyledAutosizer = styled((props: AutoSizerProps) => <AutoSizer {...props} />)(
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

export interface LiftTableProps {
  lifts: Lift[];
  liftStates: Record<string, LiftState>;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

interface LiftListRendererProps extends ListChildComponentProps {
  data: LiftTableProps;
}

export interface LiftRowProps {
  lift: Lift;
  doorState: number;
  currentMode: number;
  currentFloor: string;
  destinationFloor: string;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

const LiftRow = React.memo(
  ({
    lift,
    doorState,
    destinationFloor,
    currentFloor,
    currentMode,
    onRequestSubmit,
  }: LiftRowProps) => {
    const [showForms, setShowForms] = React.useState(false);
    const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStylesClasses;

    const doorModeLabelClasses = React.useCallback((doorState: number): string => {
      switch (doorState) {
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

    return (
      <TableRow aria-label={`${lift.name}`} component="div" className={classes.tableRow}>
        <ItemTableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
          title={lift.name}
        >
          {lift.name}
        </ItemTableCell>
        <ItemTableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
        >
          {liftModeToString(currentMode)}
        </ItemTableCell>
        <ItemTableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
        >
          {currentFloor}
        </ItemTableCell>
        <ItemTableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
        >
          {destinationFloor}
        </ItemTableCell>
        <ItemTableCell
          component="div"
          variant="body"
          className={clsx(doorModeLabelClasses(doorState), classes.tableCell, fixedTableCell)}
        >
          {doorStateToString(doorState)}
        </ItemTableCell>
        <ItemTableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedLastTableCell)}
        >
          <Button
            variant="contained"
            color="primary"
            fullWidth
            size="small"
            onClick={() => setShowForms(true)}
          >
            Request Form
          </Button>
          <LiftRequestFormDialog
            lift={lift}
            availableDoorModes={requestDoorModes}
            availableRequestTypes={requestModes}
            showFormDialog={showForms}
            onRequestSubmit={onRequestSubmit}
            onClose={() => setShowForms(false)}
          />
        </ItemTableCell>
      </TableRow>
    );
  },
);

const LiftListRenderer = ({ data, index }: LiftListRendererProps) => {
  const lift = data.lifts[index];
  const liftState = data.liftStates[lift.name];

  return (
    <LiftRow
      lift={lift}
      doorState={liftState?.door_state}
      currentMode={liftState?.current_mode}
      currentFloor={liftState?.current_floor}
      destinationFloor={liftState?.destination_floor}
      onRequestSubmit={data.onRequestSubmit}
    />
  );
};

export const LiftTable = ({ lifts, liftStates, onRequestSubmit }: LiftTableProps): JSX.Element => {
  const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStylesClasses;
  return (
    <StyledAutosizer disableHeight>
      {({ width }) => {
        return (
          <Table component="div" stickyHeader size="small" aria-label="lift-table">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow}>
                <ItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Lift Name
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Op. Mode
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Current Floor
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Destination
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Doors State
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedLastTableCell)}
                >
                  Request Form
                </ItemTableCell>
              </TableRow>
            </TableHead>
            <TableBody component="div">
              <FixedSizeList
                itemSize={43}
                itemCount={lifts.length}
                height={200}
                width={width}
                itemData={{
                  lifts,
                  liftStates,
                  width,
                  onRequestSubmit,
                }}
              >
                {LiftListRenderer}
              </FixedSizeList>
            </TableBody>
          </Table>
        );
      }}
    </StyledAutosizer>
  );
};
