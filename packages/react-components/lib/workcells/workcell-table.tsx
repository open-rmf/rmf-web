import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import { Dispenser } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { dispenserModeToString, workCellTableCellConfig } from './utils';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import clsx from 'clsx';
import AutoSizer from 'react-virtualized-auto-sizer';

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
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
}));

export interface WorkcellTableProps {
  workcells: Dispenser[];
  workcellStates: Record<string, RmfModels.DispenserState>;
}

export interface WorkcellFixListDataProps extends WorkcellTableProps {
  width: number;
}

export interface WorkcellRowProps extends ListChildComponentProps {
  data: WorkcellFixListDataProps;
}

const WorkcellRow = React.memo(({ data, index, style }: WorkcellRowProps) => {
  const classes = useStyles();
  const workcell = data.workcells[index];
  const state: RmfModels.DispenserState | RmfModels.IngestorState | undefined =
    data.workcellStates[workcell.guid];
  const mode = state?.mode;
  const requestGuidQueue = state?.request_guid_queue;
  const secondsRemaining = state?.seconds_remaining;
  const width = data.width;

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
            className={classes.tableCell}
            style={{
              minWidth: width * workCellTableCellConfig.dispenserName - 40,
              height: workCellTableCellConfig.rowHeight,
            }}
            title={workcell?.guid}
          >
            {workcell.guid}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={clsx(dispenserModeLabelClasses(mode), classes.tableCell)}
            style={{
              minWidth: width * workCellTableCellConfig.opMode - 40,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {dispenserModeToString(mode)}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={classes.tableCell}
            style={{
              minWidth: width * workCellTableCellConfig.numQueueRequest - 40,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {requestGuidQueue.length}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={classes.tableCell}
            style={{
              minWidth: width * workCellTableCellConfig.requestQueueId - 40,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {requestGuidQueue}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={classes.tableCell}
            style={{
              minWidth: width * workCellTableCellConfig.secRemaining - 32,
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
            className={classes.tableCell}
            style={{
              minWidth: width * workCellTableCellConfig.dispenserName - 40,
              height: workCellTableCellConfig.rowHeight,
            }}
            title={workcell?.guid}
          >
            {workcell.guid}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={classes.tableCell}
            style={{
              minWidth: width * workCellTableCellConfig.opMode - 40,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {'NA'}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={classes.tableCell}
            style={{
              minWidth: width * workCellTableCellConfig.numQueueRequest - 40,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {'NA'}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={classes.tableCell}
            style={{
              minWidth: width * workCellTableCellConfig.requestQueueId - 40,
              height: workCellTableCellConfig.rowHeight,
            }}
          >
            {'NA'}
          </TableCell>
          <TableCell
            component="div"
            variant="head"
            className={classes.tableCell}
            style={{
              minWidth: width * workCellTableCellConfig.secRemaining - 32,
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
    <AutoSizer disableHeight>
      {({ width }) => {
        return (
          <Table component="div" stickyHeader size="small" aria-label="workcell-table">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow}>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * workCellTableCellConfig.dispenserName - 40,
                    height: workCellTableCellConfig.rowHeight,
                  }}
                >
                  Dispenser Name
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * workCellTableCellConfig.opMode - 40,
                    height: workCellTableCellConfig.rowHeight,
                  }}
                >
                  Op. Mode
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * workCellTableCellConfig.numQueueRequest - 40,
                    height: workCellTableCellConfig.rowHeight,
                  }}
                >
                  No. Queued Requests
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * workCellTableCellConfig.requestQueueId - 40,
                    height: workCellTableCellConfig.rowHeight,
                  }}
                >
                  Request Queue ID
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={{
                    minWidth: width * workCellTableCellConfig.secRemaining - 32,
                    height: workCellTableCellConfig.rowHeight,
                  }}
                >
                  Seconds Remaining
                </TableCell>
              </TableRow>
            </TableHead>
            <TableBody component="div">
              <FixedSizeList
                itemSize={43}
                itemCount={workcells.length}
                height={200}
                width={width}
                itemData={{
                  workcells,
                  workcellStates,
                  width,
                }}
              >
                {WorkcellRow}
              </FixedSizeList>
            </TableBody>
          </Table>
        );
      }}
    </AutoSizer>
  );
};
