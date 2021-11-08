import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  TableProps,
  styled,
} from '@mui/material';
import { Dispenser } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { dispenserModeToString } from './utils';

const classes = {
  dispenserLabelIdle: 'workcell-dispenser-label-idle',
  dispenserLabelBusy: 'workcell-dispenser-label-busy',
  dispenserLabelOffline: 'workcell-offline-label',
};
const StyledTable = styled((props: TableProps) => <Table {...props} />)(({ theme }) => ({
  [`& .${classes.dispenserLabelIdle}`]: {
    color: theme.palette.success.main,
  },
  [`& .${classes.dispenserLabelBusy}`]: {
    color: theme.palette.error.main,
  },
  [`& .${classes.dispenserLabelOffline}`]: {
    color: theme.palette.warning.main,
  },
}));

export interface WorkcellTableProps {
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
    const dispenserModeLabelClasses = React.useCallback(
      (mode: number): string => {
        switch (mode) {
          case RmfModels.DispenserState.IDLE:
            return `${classes.dispenserLabelIdle}`;
          case RmfModels.DispenserState.BUSY:
            return `${classes.dispenserLabelBusy}`;
          case RmfModels.DispenserState.OFFLINE:
            return `${classes.dispenserLabelOffline}`;
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
    <StyledTable stickyHeader size="small" aria-label="workcell-table">
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
    </StyledTable>
  );
};
