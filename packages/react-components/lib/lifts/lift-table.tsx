import { Button, Table, TableBody, TableHead, TableRow, styled } from '@mui/material';
import React from 'react';
import * as RmfModels from 'rmf-models';
import LiftRequestFormDialog from './lift-request-form-dialog';
import { doorStateToString, liftModeToString, requestDoorModes, requestModes } from './lift-utils';
import { useFixedTableCellStylesClasses, StyledItemTableCell } from '../utils';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import clsx from 'clsx';
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
    onRequestSubmit,
  }: LiftRowProps) => {
    const [showForms, setShowForms] = React.useState(false);
    const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStylesClasses;

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
      <TableRow aria-label={`${lift.name}`} component="div" className={classes.tableRow}>
        <StyledItemTableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
          title={lift.name}
        >
          {lift.name}
        </StyledItemTableCell>
        <StyledItemTableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
        >
          {liftModeToString(currentMode)}
        </StyledItemTableCell>
        <StyledItemTableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
        >
          {currentFloor}
        </StyledItemTableCell>
        <StyledItemTableCell
          component="div"
          variant="body"
          className={clsx(classes.tableCell, fixedTableCell)}
        >
          {destinationFloor}
        </StyledItemTableCell>
        <StyledItemTableCell
          component="div"
          variant="body"
          className={clsx(doorModeLabelClasses(doorState), classes.tableCell, fixedTableCell)}
        >
          {doorStateToString(doorState)}
        </StyledItemTableCell>
        <StyledItemTableCell
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
        </StyledItemTableCell>
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
                <StyledItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Lift Name
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Op. Mode
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Current Floor
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Destination
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Doors State
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="body"
                  className={clsx(classes.tableCell, fixedLastTableCell)}
                >
                  Request Form
                </StyledItemTableCell>
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
