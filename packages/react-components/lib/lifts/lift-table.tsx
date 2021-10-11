import {
  Button,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  TableProps,
  styled,
} from '@material-ui/core';
import React from 'react';
import * as RmfModels from 'rmf-models';
import LiftRequestFormDialog from './lift-request-form-dialog';
import { doorStateToString, liftModeToString, requestDoorModes, requestModes } from './lift-utils';

const classes = {
  doorLabelOpen: 'lift-table-doorlabelopen',
  doorLabelClosed: 'lift-table-doorlabelclosed',
  doorLabelMoving: 'lift-table-doorlabelmoving',
};
const LiftTableRoot = styled((props: TableProps) => <Table {...props} />)(({ theme }) => ({
  [`& .${classes.doorLabelOpen}`]: {
    color: theme.palette.success.main,
  },
  [`& .${classes.doorLabelClosed}`]: {
    color: theme.palette.error.main,
  },
  [`& .${classes.doorLabelMoving}`]: {
    color: theme.palette.warning.main,
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

export interface LiftRowProps {
  lift: RmfModels.Lift;
  doorState?: number;
  destinationFloor?: string;
  currentFloor?: string;
  currentMode?: number;
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
      <TableRow aria-label={`${lift.name}`}>
        <TableCell>{lift.name}</TableCell>
        <TableCell>{liftModeToString(currentMode)}</TableCell>
        <TableCell>{currentFloor}</TableCell>
        <TableCell>{destinationFloor}</TableCell>
        <TableCell className={doorModeLabelClasses(doorState)}>
          {doorStateToString(doorState)}
        </TableCell>
        <TableCell>
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

export const LiftTable = ({ lifts, liftStates, onRequestSubmit }: LiftTableProps): JSX.Element => {
  return (
    <LiftTableRoot stickyHeader size="small" aria-label="lift-table">
      <TableHead>
        <TableRow>
          <TableCell>Lift Name</TableCell>
          <TableCell>Op. Mode</TableCell>
          <TableCell>Current Floor</TableCell>
          <TableCell>Destination</TableCell>
          <TableCell>Doors State</TableCell>
          <TableCell>Request Form</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {lifts.map((lift, i) => {
          const state: RmfModels.LiftState | undefined = liftStates[lift.name];
          return (
            <LiftRow
              lift={lift}
              doorState={state?.door_state}
              destinationFloor={state?.destination_floor}
              currentFloor={state?.current_floor}
              currentMode={state?.current_mode}
              key={`${lift.name}_${i}`}
              onRequestSubmit={onRequestSubmit}
            />
          );
        })}
      </TableBody>
    </LiftTableRoot>
  );
};
