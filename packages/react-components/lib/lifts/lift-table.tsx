import {
  Button,
  makeStyles,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
} from '@material-ui/core';
import React from 'react';
import * as RmfModels from 'rmf-models';
import LiftRequestFormDialog from './lift-request-form-dialog';
import { doorStateToString, liftModeToString, requestDoorModes, requestModes } from './lift-utils';
import { useFixedTableCellStyles } from '../utils';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import clsx from 'clsx';
import AutoSizer from 'react-virtualized-auto-sizer';

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

export interface LiftTableProps {
  lifts: RmfModels.Lift[];
  liftStates: Record<string, RmfModels.LiftState>;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: RmfModels.Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

interface LiftListRendererProps extends ListChildComponentProps {
  data: LiftTableProps;
}

export interface LiftRowProps {
  lift: RmfModels.Lift;
  doorState: number;
  currentMode: number;
  currentFloor: string;
  destinationFloor: string;
  style: React.CSSProperties;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: RmfModels.Lift,
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
    style,
    onRequestSubmit,
  }: LiftRowProps) => {
    const classes = useStyles();
    const [showForms, setShowForms] = React.useState(false);
    const fixedtableCellClass = useFixedTableCellStyles().fixedTableCell;
    const lastTableCellClass = useFixedTableCellStyles().fixedLastTableCell;

    const doorModeLabelClasses = React.useCallback(
      (doorState: number): string => {
        switch (doorState) {
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

    return (
      <TableRow
        aria-label={`${lift.name}`}
        component="div"
        className={classes.tableRow}
        style={style}
      >
        <TableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedtableCellClass)}
          title={lift.name}
        >
          {lift.name}
        </TableCell>
        <TableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedtableCellClass)}
        >
          {liftModeToString(currentMode)}
        </TableCell>
        <TableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedtableCellClass)}
        >
          {currentFloor}
        </TableCell>
        <TableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedtableCellClass)}
        >
          {destinationFloor}
        </TableCell>
        <TableCell
          component="div"
          variant="body"
          className={clsx(doorModeLabelClasses(doorState), classes.tableCell, fixedtableCellClass)}
        >
          {doorStateToString(doorState)}
        </TableCell>
        <TableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, lastTableCellClass)}
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
        </TableCell>
      </TableRow>
    );
  },
);

const LiftListRenderer = ({ data, index, style }: LiftListRendererProps) => {
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
      style={style}
    />
  );
};

export const LiftTable = ({ lifts, liftStates, onRequestSubmit }: LiftTableProps): JSX.Element => {
  const classes = useStyles();
  const fixedtableCellClass = useFixedTableCellStyles().fixedTableCell;
  const lastTableCellClass = useFixedTableCellStyles().fixedLastTableCell;
  return (
    <AutoSizer disableHeight>
      {({ width }) => {
        return (
          <Table component="div" stickyHeader size="small" aria-label="lift-table">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow}>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedtableCellClass)}
                >
                  Lift Name
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedtableCellClass)}
                >
                  Op. Mode
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedtableCellClass)}
                >
                  Current Floor
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedtableCellClass)}
                >
                  Destination
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedtableCellClass)}
                >
                  Doors State
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, lastTableCellClass)}
                >
                  Request Form
                </TableCell>
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
    </AutoSizer>
  );
};
