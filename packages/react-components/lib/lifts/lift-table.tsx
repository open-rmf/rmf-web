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
import { LeafletContext } from 'react-leaflet';
import LiftRequestFormDialog from './lift-request-form-dialog';
import {
  doorStateToString,
  liftModeToString,
  onLiftClick,
  requestDoorModes,
  requestModes,
} from './lift-utils';
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

export interface LiftTableProps {
  lifts: RmfModels.Lift[];
  liftStates: Record<string, RmfModels.LiftState>;
  leafletMap?: LeafletContext;
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
  destinationFloor?: string;
  currentFloor?: string;
  currentMode?: number;
  leafletMap?: LeafletContext;
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
    leafletMap,
    onRequestSubmit,
  }: LiftRowProps) => {
    const classes = useStyles();
    const [showForms, setShowForms] = React.useState(false);
    const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStyles();

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
        onClick={() => onLiftClick(lift, leafletMap)}
      >
        <TableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
          title={lift.name}
        >
          {lift.name}
        </TableCell>
        <TableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
        >
          {liftModeToString(currentMode)}
        </TableCell>
        <TableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
        >
          {currentFloor}
        </TableCell>
        <TableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
        >
          {destinationFloor}
        </TableCell>
        <TableCell
          component="div"
          variant="body"
          className={clsx(doorModeLabelClasses(doorState), classes.tableCell, fixedTableCell)}
        >
          {doorStateToString(doorState)}
        </TableCell>
        <TableCell
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
        </TableCell>
      </TableRow>
    );
  },
);

const LiftListRenderer = ({ data, index }: LiftListRendererProps) => {
  const lift = data.lifts[index];
  const liftState = data.liftStates[lift.name];
  const leafletMap = data.leafletMap;

  return (
    <LiftRow
      lift={lift}
      doorState={liftState?.door_state}
      currentMode={liftState?.current_mode}
      currentFloor={liftState?.current_floor}
      destinationFloor={liftState?.destination_floor}
      onRequestSubmit={data.onRequestSubmit}
      key={`${lift.name}`}
      leafletMap={leafletMap}
    />
  );
};

export const LiftTable = ({ lifts, liftStates, onRequestSubmit }: LiftTableProps): JSX.Element => {
  const classes = useStyles();
  const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStyles();
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
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Lift Name
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Op. Mode
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Current Floor
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Destination
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Doors State
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedLastTableCell)}
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
