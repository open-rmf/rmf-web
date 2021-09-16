import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  TableProps,
  styled,
} from '@material-ui/core';
import { Dispenser } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { dispenserModeToString } from './utils';

const classes = {
  dispenserLabelIdle: 'workcell-dispenser-label-idle',
  dispenserLabelBusy: 'workcell-dispenser-label-busy',
  dispenserLabelOffline: 'workcell-offline-label',
};
const WorkCellTableRoot = styled((props: TableProps) => <Table {...props} />)(({ theme }) => ({
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
  workcellState: RmfModels.DispenserState;
}

const WorkcellRow = React.memo(({ workcell, workcellState }: WorkcellRowProps) => {
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
      {workcellState ? (
        <React.Fragment>
          <TableCell>{workcell.guid}</TableCell>
          <TableCell className={dispenserModeLabelClasses(workcellState.mode)}>
            {dispenserModeToString(workcellState.mode)}
          </TableCell>
          <TableCell>{workcellState.request_guid_queue.length}</TableCell>
          <TableCell>{workcellState.request_guid_queue}</TableCell>
          <TableCell>{workcellState.seconds_remaining}</TableCell>
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
});

export const WorkcellTable = ({ workcells, workcellStates }: WorkcellTableProps): JSX.Element => {
  return (
    <WorkCellTableRoot stickyHeader size="small" aria-label="workcell-table">
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
          return (
            <WorkcellRow
              workcell={workcell}
              workcellState={workcellStates[workcell.guid]}
              key={workcell.guid}
            />
          );
        })}
      </TableBody>
    </WorkCellTableRoot>
  );
};
