import { Table, TableBody, TableHead, TableRow } from '@mui/material';
import clsx from 'clsx';
import React from 'react';
import { DispenserState as RmfDispenserState } from 'rmf-models';
import { Workcell, WorkcellState } from '.';
import { ItemTableCell, useFixedTableCellStylesClasses } from '../utils';
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
      <TableRow aria-label={`${workcell.guid}`} className={classes.tableRow} component="div">
        {mode !== undefined && requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
          <React.Fragment>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </ItemTableCell>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(dispenserModeLabelClasses(mode), classes.tableCell, fixedTableCell)}
            >
              {dispenserModeToString(mode)}
            </ItemTableCell>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {requestGuidQueue.length}
            </ItemTableCell>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {requestGuidQueue}
            </ItemTableCell>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {secondsRemaining}
            </ItemTableCell>
          </React.Fragment>
        ) : (
          <React.Fragment>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </ItemTableCell>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </ItemTableCell>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </ItemTableCell>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </ItemTableCell>
            <ItemTableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
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
    <Table component="div" size="small" aria-label="workcell-table">
      <TableHead component="div">
        <TableRow component="div" className={classes.tableRow}>
          <ItemTableCell component="div" variant="head">
            Dispenser Name
          </ItemTableCell>
          <ItemTableCell component="div" variant="head">
            Op. Mode
          </ItemTableCell>
          <ItemTableCell component="div" variant="head">
            No. Queued Requests
          </ItemTableCell>
          <ItemTableCell component="div" variant="head">
            Request Queue ID
          </ItemTableCell>
          <ItemTableCell component="div" variant="head">
            Seconds Remaining
          </ItemTableCell>
        </TableRow>
      </TableHead>
      <TableBody component="div">
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
