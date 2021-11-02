import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import { Dispenser } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { dispenserModeToString } from './utils';
import { tableCellStyle } from '../utils';
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

interface WorkcellListRendererProps extends ListChildComponentProps {
  data: WorkcellTableProps;
}

export interface WorkcellRowProps {
  workcell: Dispenser;
  mode?: number;
  requestGuidQueue?: string[];
  secondsRemaining?: number;
  style: React.CSSProperties;
}

const WorkcellRow = React.memo(
  ({ workcell, mode, requestGuidQueue, secondsRemaining, style }: WorkcellRowProps) => {
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
              style={tableCellStyle('1')}
              title={workcell.guid}
            >
              {workcell.guid}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={clsx(dispenserModeLabelClasses(mode), classes.tableCell)}
              style={tableCellStyle('1')}
            >
              {dispenserModeToString(mode)}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={classes.tableCell}
              style={tableCellStyle('1')}
            >
              {requestGuidQueue.length}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={classes.tableCell}
              style={tableCellStyle('1')}
            >
              {requestGuidQueue}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={classes.tableCell}
              style={tableCellStyle('1')}
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
              style={tableCellStyle('1')}
              title={workcell.guid}
            >
              {workcell.guid}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={classes.tableCell}
              style={tableCellStyle('1')}
            >
              {'NA'}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={classes.tableCell}
              style={tableCellStyle('1')}
            >
              {'NA'}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={classes.tableCell}
              style={tableCellStyle('1')}
            >
              {'NA'}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={classes.tableCell}
              style={tableCellStyle('1')}
            >
              {'NA'}
            </TableCell>
          </React.Fragment>
        )}
      </TableRow>
    );
  },
);

const WorkcellListRenderer = ({ data, index, style }: WorkcellListRendererProps) => {
  const workcell = data.workcells[index];
  const workcellState: RmfModels.DispenserState | RmfModels.IngestorState | undefined =
    data.workcellStates[workcell.guid];

  return (
    <WorkcellRow
      workcell={workcell}
      mode={workcellState?.mode}
      requestGuidQueue={workcellState?.request_guid_queue}
      secondsRemaining={workcellState?.seconds_remaining}
      style={style}
    />
  );
};

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
                  style={tableCellStyle('1')}
                >
                  Dispenser Name
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1')}
                >
                  Op. Mode
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1')}
                >
                  No. Queued Requests
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1')}
                >
                  Request Queue ID
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={classes.tableCell}
                  style={tableCellStyle('1')}
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
                {WorkcellListRenderer}
              </FixedSizeList>
            </TableBody>
          </Table>
        );
      }}
    </AutoSizer>
  );
};
