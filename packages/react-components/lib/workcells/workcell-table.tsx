import {
  makeStyles,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
} from '@material-ui/core';
import { Dispenser } from 'api-client';
import React from 'react';
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
  tableContainer: {
    maxHeight: '25vh',
  },
}));

export interface WorkcellTableProps {
  type?: string;
  workcells: Dispenser[];
  workcellStates: Record<string, RmfModels.DispenserState>;
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

export const WorkcellTable = ({
  type,
  workcells,
  workcellStates,
}: WorkcellTableProps): JSX.Element => {
  const classes = useStyles();

  return (
    <TableContainer className={classes.tableContainer}>
      <Table stickyHeader size="small" aria-label="workcell-table">
        <TableHead>
          <TableRow>
            <TableCell>{type} Name</TableCell>
            <TableCell>Op. Mode</TableCell>
            <TableCell>No. Queued Requests</TableCell>
            <TableCell>Request Queue ID</TableCell>
            <TableCell>Seconds Remaining</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {workcells.map((workcell) => {
            const state: RmfModels.DispenserState | RmfModels.IngestorState | undefined =
              workcellStates[workcell.guid];
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
    </TableContainer>
  );
};
