import React from 'react';
import { Dispenser } from 'api-client';
import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import * as RmfModels from 'rmf-models';
import { dispenserModeToString } from './utils';

const useStyles = makeStyles((theme) => ({
  dispenserLabelIdle: {
    color: theme.palette.success.main,
  },
  dispenserLabelBusy: {
    color: theme.palette.error.main,
  },
  offlineLabelOffline: {
    color: theme.palette.warning.main,
  },
}));

export interface DispenserTableProps {
  dispensers: Dispenser[];
  dispenserStates: Record<string, RmfModels.DispenserState>;
}

export interface DispenserRowProps {
  dispenser: Dispenser;
  dispenserState: RmfModels.DispenserState;
}

const DispenserRow = (props: DispenserRowProps) => {
  const { dispenser, dispenserState } = props;
  const classes = useStyles();

  const dispenserModeLabelClasses = React.useCallback(
    (mode: number): string => {
      switch (mode) {
        case RmfModels.DispenserState.IDLE:
          return `${classes.dispenserLabelIdle}`;
        case RmfModels.DispenserState.BUSY:
          return `${classes.dispenserLabelBusy}`;
        case RmfModels.DispenserState.OFFLINE:
          return `${classes.offlineLabelOffline}`;
        default:
          return '';
      }
    },
    [classes],
  );

  return (
    <>
      <TableRow aria-label={`${dispenser.guid}`}>
        {dispenserState ? (
          <React.Fragment>
            <TableCell>{dispenser.guid}</TableCell>
            <TableCell className={dispenserModeLabelClasses(dispenserState.mode)}>
              {dispenserModeToString(dispenserState.mode)}
            </TableCell>
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
