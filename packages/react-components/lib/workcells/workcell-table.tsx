import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import type { Dispenser, DispenserState, IngestorState } from 'api-client';
import React from 'react';
import { DispenserState as RmfDispenserState } from 'rmf-models';
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

export interface WorkcellTableProps {
  workcells: Dispenser[];
  workcellStates: Record<string, DispenserState | IngestorState>;
}

export interface WorkcellRowProps {
  workcell: Dispenser;
  mode?: number;
  requestGuidQueue?: string[];
  secondsRemaining?: number;
}

const WorkcellRow = React.memo(
  ({ workcell, mode, requestGuidQueue, secondsRemaining }: WorkcellRowProps) => {
    const classes = useStyles();

    const dispenserModeLabelClasses = React.useCallback(
      (mode: number): string => {
        switch (mode) {
          case RmfDispenserState.IDLE:
            return `${classes.dispenserLabelIdle}`;
          case RmfDispenserState.BUSY:
            return `${classes.dispenserLabelBusy}`;
          case RmfDispenserState.OFFLINE:
            return `${classes.offlineLabelOffline}`;
          default:
            return '';
        }
      },
      [classes],
    );

    return (
      <TableRow aria-label={`${workcell.guid}`}>
        {mode !== undefined && requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
          <React.Fragment>
            <TableCell>{workcell.guid}</TableCell>
            <TableCell className={dispenserModeLabelClasses(mode)}>
              {dispenserModeToString(mode)}
            </TableCell>
            <TableCell>{requestGuidQueue.length}</TableCell>
            <TableCell>{requestGuidQueue}</TableCell>
            <TableCell>{secondsRemaining}</TableCell>
          </React.Fragment>
        ) : (
          <React.Fragment>
            <TableCell>{workcell.guid}</TableCell>
            <TableCell>{'NA'}</TableCell>
            <TableCell>{'NA'}</TableCell>
            <TableCell>{'NA'}</TableCell>
            <TableCell>{'NA'}</TableCell>
          </React.Fragment>
        )}
      </TableRow>
    );
  },
);

export const WorkcellTable = ({ workcells, workcellStates }: WorkcellTableProps): JSX.Element => {
  return (
    <Table stickyHeader size="small" aria-label="workcell-table">
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
        {workcells.map((workcell) => {
          const state: DispenserState | IngestorState | undefined = workcellStates[workcell.guid];
          return (
            <WorkcellRow
              key={workcell.guid}
              workcell={workcell}
              mode={state?.mode}
              requestGuidQueue={state?.request_guid_queue}
              secondsRemaining={state?.seconds_remaining}
            />
          );
        })}
      </TableBody>
    </Table>
  );
};
