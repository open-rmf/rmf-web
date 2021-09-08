import { Button, Table, TableBody, TableCell, TableHead, TableRow } from '@mui/material';
import { makeStyles } from '@mui/styles';
import React from 'react';
import * as RmfModels from 'rmf-models';
import LiftRequestFormDialog from './lift-request-form-dialog';
import { doorStateToString, liftModeToString, requestDoorModes, requestModes } from './lift-utils';

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
  liftState: RmfModels.LiftState;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: RmfModels.Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

const LiftRow = (props: LiftRowProps) => {
  const { lift, liftState, onRequestSubmit } = props;
  const classes = useStyles();

  const [showForms, setShowForms] = React.useState(false);

  const doorModeLabelClasses = React.useCallback(
    (liftState?: RmfModels.LiftState): string => {
      switch (liftState?.door_state) {
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
      <TableCell>{liftModeToString(liftState.current_mode)}</TableCell>
      <TableCell>{liftState.current_floor}</TableCell>
      <TableCell>{liftState.destination_floor}</TableCell>
      <TableCell className={doorModeLabelClasses(liftState)}>
        {doorStateToString(liftState.door_state)}
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
};

export const LiftTable = ({ lifts, liftStates, onRequestSubmit }: LiftTableProps): JSX.Element => {
  return (
    <Table stickyHeader size="small" aria-label="lift-table">
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
          return (
            <LiftRow
              lift={lift}
              liftState={liftStates[lift.name]}
              key={`${lift.name}_${i}`}
              onRequestSubmit={onRequestSubmit}
            />
          );
        })}
      </TableBody>
    </Table>
  );
};
