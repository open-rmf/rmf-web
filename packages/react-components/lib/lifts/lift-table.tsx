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

export interface LiftFixListDataProps extends LiftTableProps {
  width: number;
}

export interface LiftListRendererProps extends ListChildComponentProps {
  data: LiftFixListDataProps;
}

export interface LiftRowProps {
  lift: RmfModels.Lift;
  liftState: RmfModels.LiftState;
  style: React.CSSProperties;
  width: number;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: RmfModels.Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

// table cell has padding of 16px left and 24px right respectively
// need to deduct 40px away from actual width
const liftTableCellConfig = {
  // column width in percent
  liftName: 0.133,
  opMode: 0.227,
  currentFloor: 0.16,
  destination: 0.152,
  doorState: 0.148,
  // last column deduct 32px
  button: 0.18,
  // row height in pixels
  rowHeight: 31,
};

const LiftRow = React.memo(({ lift, liftState, style, width, onRequestSubmit }: LiftRowProps) => {
  const classes = useStyles();
  const doorState = liftState?.door_state;
  const destinationFloor = liftState?.destination_floor;
  const currentFloor = liftState?.current_floor;
  const currentMode = liftState?.current_mode;
  const [showForms, setShowForms] = React.useState(false);

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
        className={classes.tableCell}
        style={{
          minWidth: width * liftTableCellConfig.liftName - 40,
          height: liftTableCellConfig.rowHeight,
        }}
        title={lift.name}
      >
        {lift.name}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={classes.tableCell}
        style={{
          minWidth: width * liftTableCellConfig.opMode - 40,
          height: liftTableCellConfig.rowHeight,
        }}
      >
        {liftModeToString(currentMode)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={classes.tableCell}
        style={{
          minWidth: width * liftTableCellConfig.currentFloor - 40,
          height: liftTableCellConfig.rowHeight,
        }}
      >
        {currentFloor}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={classes.tableCell}
        style={{
          minWidth: width * liftTableCellConfig.destination - 40,
          height: liftTableCellConfig.rowHeight,
        }}
      >
        {destinationFloor}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={clsx(doorModeLabelClasses(doorState), classes.tableCell)}
        style={{
          minWidth: width * liftTableCellConfig.doorState - 40,
          height: liftTableCellConfig.rowHeight,
        }}
      >
        {doorStateToString(doorState)}
      </TableCell>
      <TableCell
        component="div"
        variant="body"
        className={classes.tableCell}
        style={{
          minWidth: width * liftTableCellConfig.button - 32,
          height: liftTableCellConfig.rowHeight,
        }}
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

const LiftListRenderer = ({ data, index, style }: LiftListRendererProps) => {
  const lift = data.lifts[index];
  const liftState = data.liftStates[lift.name];

  return (
    <LiftRow
      lift={lift}
      liftState={liftState}
      onRequestSubmit={data.onRequestSubmit}
      style={style}
      width={data.width}
    />
  );
};

export const LiftTable = ({ lifts, liftStates, onRequestSubmit }: LiftTableProps): JSX.Element => {
  const classes = useStyles();
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
                  className={classes.tableCell}
                  style={{
                    minWidth: width * liftTableCellConfig.liftName - 40,
                    height: liftTableCellConfig.rowHeight,
                  }}
                >
                  Lift Name
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * liftTableCellConfig.opMode - 40,
                    height: liftTableCellConfig.rowHeight,
                  }}
                >
                  Op. Mode
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * liftTableCellConfig.currentFloor - 40,
                    height: liftTableCellConfig.rowHeight,
                  }}
                >
                  Current Floor
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * liftTableCellConfig.destination - 40,
                    height: liftTableCellConfig.rowHeight,
                  }}
                >
                  Destination
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * liftTableCellConfig.doorState - 40,
                    height: liftTableCellConfig.rowHeight,
                  }}
                >
                  Doors State
                </TableCell>
                <TableCell
                  component="div"
                  variant="body"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * liftTableCellConfig.button - 32,
                    height: liftTableCellConfig.rowHeight,
                  }}
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
