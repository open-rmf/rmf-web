import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import { Dispenser } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { dispenserModeToString, workCellTableCellConfig } from './utils';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import clsx from 'clsx';

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
  expandingCell: {
    flex: 1,
  },
  tableRow: {
    display: 'flex',
    flexDirection: 'row',
    flexWrap: 'nowrap',
    alignItems: 'center',
    boxSizing: 'border-box',
    minWidth: '100%',
    width: '100%',
  },
  tableCell: {
    display: 'block',
    flexGrow: 0,
    flexShrink: 0,
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
  tableBody: {
    width: '100%',
  },
}));

export interface WorkcellTableProps {
  workcells: Dispenser[];
  workcellStates: Record<string, RmfModels.DispenserState>;
}

export interface WorkcellRowProps extends ListChildComponentProps {
  data: WorkcellTableProps;
}

const WorkcellRow = React.memo(({ data, index, style }: WorkcellRowProps) => {
  const classes = useStyles();
  const workcell = data.workcells[index];
  const state: RmfModels.DispenserState | RmfModels.IngestorState | undefined =
    data.workcellStates[workcell.guid];
  const mode = state?.mode;
  const requestGuidQueue = state?.request_guid_queue;
  const secondsRemaining = state?.seconds_remaining;

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
    <TableRow
      aria-label={`${workcell.guid}`}
      className={classes.tableRow}
      component="div"
      style={style}
    >
      {mode !== undefined && requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
        <React.Fragment>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.dispenserName,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {workcell.guid}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(
              dispenserModeLabelClasses(mode),
              classes.tableCell,
              classes.expandingCell,
            )}
            style={{
              minWidth: workCellTableCellConfig.opMode,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {dispenserModeToString(mode)}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.numQueueRequest,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {requestGuidQueue.length}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.requestQueueId,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {requestGuidQueue}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.secRemaining,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {secondsRemaining}
          </TableCell>
        </React.Fragment>
      ) : (
        <React.Fragment>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.dispenserName,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {workcell.guid}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.opMode,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {'NA'}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.numQueueRequest,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {'NA'}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.requestQueueId,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {'NA'}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.secRemaining,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {'NA'}
          </TableCell>
        </React.Fragment>
      )}
    </TableRow>
  );
});

export const WorkcellTable = ({ workcells, workcellStates }: WorkcellTableProps): JSX.Element => {
  const classes = useStyles();
  return (
    <Table component="div" stickyHeader size="small" aria-label="workcell-table">
      <TableHead component="div">
        <TableRow component="div" className={classes.tableRow}>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.dispenserName,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            Dispenser Name
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.opMode,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            Op. Mode
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.numQueueRequest,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            No. Queued Requests
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.requestQueueId,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            Request Queue ID
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(classes.tableCell, classes.expandingCell)}
            style={{
              minWidth: workCellTableCellConfig.secRemaining,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            Seconds Remaining
          </TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        <FixedSizeList
          itemSize={43}
          itemCount={workcells.length}
          height={200}
          width={760}
          itemData={{
            workcells,
            workcellStates,
          }}
        >
          {WorkcellRow}
        </FixedSizeList>
      </TableBody>
    </Table>
  );
};
