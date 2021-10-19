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
import {
  doorStateToString,
  liftModeToString,
  requestDoorModes,
  requestModes,
  liftTableCellConfig,
} from './lift-utils';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import clsx from 'clsx';

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

export interface LiftRowProps extends ListChildComponentProps {
  data: LiftTableProps;
}

const LiftRow = React.memo(({ data, index, style }: LiftRowProps) => {
  const classes = useStyles();
  const lift = data.lifts[index];
  const liftState = data.liftStates[lift.name];
  const doorState = liftState?.door_state;
  const destinationFloor = liftState?.destination_floor;
  const currentFloor = liftState?.current_floor;
  const currentMode = liftState?.current_mode;
  const onRequestSubmit = data.onRequestSubmit;
  const [showForms, setShowForms] = React.useState(false);

  const doorModeLabelClasses = React.useCallback(
    (doorState?: number): string => {
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
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{ minWidth: liftTableCellConfig.liftName, height: liftTableCellConfig.rowHeight }}
      >
        {lift.name}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{ minWidth: liftTableCellConfig.opMode, height: liftTableCellConfig.rowHeight }}
      >
        {liftModeToString(currentMode)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{
          minWidth: liftTableCellConfig.currentFloor,
          height: liftTableCellConfig.rowHeight,
        }}
      >
        {currentFloor}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{ minWidth: liftTableCellConfig.destination, height: liftTableCellConfig.rowHeight }}
      >
        {destinationFloor}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(doorModeLabelClasses(doorState), classes.tableCell, classes.expandingCell)}
        style={{ minWidth: liftTableCellConfig.doorState, height: liftTableCellConfig.rowHeight }}
      >
        {doorStateToString(doorState)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(classes.tableCell, classes.expandingCell)}
        style={{ minWidth: liftTableCellConfig.button, height: liftTableCellConfig.rowHeight }}
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
});

export const LiftTable = ({ lifts, liftStates, onRequestSubmit }: LiftTableProps): JSX.Element => {
  const classes = useStyles();
  return (
    <Table component="div" stickyHeader size="small" aria-label="lift-table">
      <TableHead component="div">
        <TableRow component="div" className={classes.tableRow}>
          <TableCell
            component="div"
            variant="body"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: liftTableCellConfig.liftName,
              height: liftTableCellConfig.rowHeight,
            }}
          >
            Lift Name
          </TableCell>
          <TableCell
            component="div"
            variant="body"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{ minWidth: liftTableCellConfig.opMode, height: liftTableCellConfig.rowHeight }}
          >
            Op. Mode
          </TableCell>
          <TableCell
            component="div"
            variant="body"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: liftTableCellConfig.currentFloor,
              height: liftTableCellConfig.rowHeight,
            }}
          >
            Current Floor
          </TableCell>
          <TableCell
            component="div"
            variant="body"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: liftTableCellConfig.destination,
              height: liftTableCellConfig.rowHeight,
            }}
          >
            Destination
          </TableCell>
          <TableCell
            component="div"
            variant="body"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: liftTableCellConfig.doorState,
              height: liftTableCellConfig.rowHeight,
            }}
          >
            Doors State
          </TableCell>
          <TableCell
            component="div"
            variant="body"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{ minWidth: liftTableCellConfig.button, height: liftTableCellConfig.rowHeight }}
          >
            Request Form
          </TableCell>
        </TableRow>
      </TableHead>
      <TableBody component="div" className={classes.tableBody}>
        <FixedSizeList
          itemSize={43}
          itemCount={lifts.length}
          height={200}
          width={760}
          itemData={{
            lifts,
            liftStates,
            onRequestSubmit,
          }}
        >
          {LiftRow}
        </FixedSizeList>
      </TableBody>
    </Table>
  );
};
