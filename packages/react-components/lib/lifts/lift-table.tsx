import React from 'react';
import * as RmfModels from 'rmf-models';
import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import { LiftTableProps, LiftRowProps, doorStateToString, liftModeToString } from './lift-utils';

const useStyles = makeStyles((theme) => ({
  taskRowHover: {
    background: theme.palette.action.hover,
  },
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

const LiftRow = (props: LiftRowProps) => {
  const { lift, liftState } = props;
  const classes = useStyles();
  const [hover, setHover] = React.useState(false);

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
    <>
      <TableRow
        className={hover ? classes.taskRowHover : ''}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell>{lift.name}</TableCell>
        <TableCell>{liftModeToString(liftState.current_mode)}</TableCell>
        <TableCell>{liftState.current_floor}</TableCell>
        <TableCell>{liftState.destination_floor}</TableCell>
        <TableCell className={doorModeLabelClasses(liftState)}>
          {doorStateToString(liftState.door_state)}
        </TableCell>
      </TableRow>
    </>
  );
};

export const LiftTable = (props: LiftTableProps) => {
  const { lifts, liftStates } = props;
  return (
    <Table stickyHeader size="small" aria-label="lift-table">
      <TableHead>
        <TableRow>
          <TableCell>Lift Name</TableCell>
          <TableCell>Op. Mode</TableCell>
          <TableCell>Current Floor</TableCell>
          <TableCell>Destination</TableCell>
          <TableCell>Doors State</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {lifts.map((lift, i) => {
          return (
            <LiftRow lift={lift} liftState={liftStates[lift.name]} key={`${lift.name}_${i}`} />
          );
        })}
      </TableBody>
    </Table>
  );
};
