import React from 'react';
import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';

import { dispenserModeToString, DispenserRowProps, DispenserTableProps } from './utils';

const useStyles = makeStyles((theme) => ({
  taskRowHover: {
    background: theme.palette.action.hover,
  },
}));

const DispenserRow = (props: DispenserRowProps) => {
  const { dispenser, dispenserState } = props;
  const classes = useStyles();
  const [hover, setHover] = React.useState(false);

  return (
    <>
      <TableRow
        className={hover ? classes.taskRowHover : ''}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        {dispenserState ? (
          <React.Fragment>
            <TableCell>{dispenser.guid}</TableCell>
            <TableCell>{dispenserModeToString(dispenserState.mode)}</TableCell>
            <TableCell>{dispenserState.request_guid_queue.length}</TableCell>
            <TableCell>{dispenserState.request_guid_queue}</TableCell>
            <TableCell>{dispenserState.seconds_remaining}</TableCell>
          </React.Fragment>
        ) : (
          <React.Fragment>
            <TableCell>{dispenser.guid}</TableCell>
            <TableCell>{'NA'}</TableCell>
            <TableCell>{'NA'}</TableCell>
            <TableCell>{'NA'}</TableCell>
            <TableCell>{'NA'}</TableCell>
          </React.Fragment>
        )}
      </TableRow>
    </>
  );
};

export const DispenserTable = (props: DispenserTableProps) => {
  const { dispensers, dispenserStates } = props;

  return (
    <Table stickyHeader size="small" aria-label="dispenser-table">
      <TableHead>
        <TableRow>
          <TableCell>Dispenser Name</TableCell>
          <TableCell>Op. Mode</TableCell>
          <TableCell>No. Queued Requests</TableCell>
          <TableCell>Request Queue ID</TableCell>
          <TableCell>Seconds Remaining</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {dispensers.map((dispenser) => {
          return (
            <DispenserRow dispenser={dispenser} dispenserState={dispenserStates[dispenser.guid]} />
          );
        })}
      </TableBody>
    </Table>
  );
};
