import { Table, TableBody, TableHead, TableRow } from '@mui/material';
import clsx from 'clsx';
import React from 'react';
import { DispenserState as RmfDispenserState } from 'rmf-models/ros/rmf_dispenser_msgs/msg';

import { ItemTableCell, useFixedTableCellStylesClasses } from '../utils/item-table';
import { Workcell, WorkcellState } from '.';
import { dispenserModeToString } from './utils';

const classes = {
  dispenserLabelIdle: 'workcell-dispenser-label-idle',
  dispenserLabelBusy: 'workcell-dispenser-label-busy',
  dispenserLabelOffline: 'workcell-offline-label',
  tableContainer: 'workcell-table-container',
  firstCell: 'workcell-table-first-cell',
  tableRow: 'workcell-table-row',
  tableCell: 'workcell-table-cell',
};

export interface WorkcellTableProps {
  workcells: Workcell[];
  workcellStates: Record<string, WorkcellState>;
}

export interface WorkcellRowProps {
  workcell: Workcell;
  mode?: number;
  requestGuidQueue?: string[];
  secondsRemaining?: number;
}

const WorkcellRow = React.memo(
  ({ workcell, mode, requestGuidQueue, secondsRemaining }: WorkcellRowProps) => {
    const { fixedTableCell } = useFixedTableCellStylesClasses;
    const dispenserModeLabelClasses = React.useCallback((mode: number): string => {
      switch (mode) {
        case RmfDispenserState.IDLE:
          return `${classes.dispenserLabelIdle}`;
        case RmfDispenserState.BUSY:
          return `${classes.dispenserLabelBusy}`;
        case RmfDispenserState.OFFLINE:
          return `${classes.dispenserLabelOffline}`;
        default:
          return '';
      }
    }, []);

    return (
      <TableRow aria-label={`${workcell.guid}`} className={classes.tableRow}>
        {mode !== undefined && requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
          <React.Fragment>
            <ItemTableCell
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </ItemTableCell>
            <ItemTableCell
              variant="body"
              className={clsx(dispenserModeLabelClasses(mode), classes.tableCell, fixedTableCell)}
            >
              {dispenserModeToString(mode)}
            </ItemTableCell>
            <ItemTableCell variant="body" className={clsx(classes.tableCell, fixedTableCell)}>
              {requestGuidQueue.length}
            </ItemTableCell>
            <ItemTableCell variant="body" className={clsx(classes.tableCell, fixedTableCell)}>
              {requestGuidQueue}
            </ItemTableCell>
            <ItemTableCell variant="body" className={clsx(classes.tableCell, fixedTableCell)}>
              {secondsRemaining}
            </ItemTableCell>
          </React.Fragment>
        ) : (
          <React.Fragment>
            <ItemTableCell
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </ItemTableCell>
            <ItemTableCell variant="body" className={clsx(classes.tableCell, fixedTableCell)}>
              {'NA'}
            </ItemTableCell>
            <ItemTableCell variant="body" className={clsx(classes.tableCell, fixedTableCell)}>
              {'NA'}
            </ItemTableCell>
            <ItemTableCell variant="body" className={clsx(classes.tableCell, fixedTableCell)}>
              {'NA'}
            </ItemTableCell>
            <ItemTableCell variant="body" className={clsx(classes.tableCell, fixedTableCell)}>
              {'NA'}
            </ItemTableCell>
          </React.Fragment>
        )}
      </TableRow>
    );
  },
);

export const WorkcellTable = ({ workcells, workcellStates }: WorkcellTableProps): JSX.Element => {
  return (
    <Table size="small" aria-label="workcell-table">
      <TableHead>
        <TableRow className={classes.tableRow}>
          <ItemTableCell variant="head">Dispenser Name</ItemTableCell>
          <ItemTableCell variant="head">Op. Mode</ItemTableCell>
          <ItemTableCell variant="head">No. Queued Requests</ItemTableCell>
          <ItemTableCell variant="head">Request Queue ID</ItemTableCell>
          <ItemTableCell variant="head">Seconds Remaining</ItemTableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {workcells.map((workcell) => {
          const workcellState: WorkcellState | undefined = workcellStates[workcell.guid];
          return (
            <WorkcellRow
              workcell={workcell}
              mode={workcellState?.mode}
              requestGuidQueue={workcellState?.request_guid_queue}
              secondsRemaining={workcellState?.seconds_remaining}
            />
          );
        })}
      </TableBody>
    </Table>
  );
};
