import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import React from 'react';
import { LeafletContext } from 'react-leaflet';
import { dispenserModeToString, onWorkcellClick, DispenserResource } from './utils';
import clsx from 'clsx';
import AutoSizer from 'react-virtualized-auto-sizer';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import { DispenserState as RmfDispenserState } from 'rmf-models';
import { Workcell, WorkcellState } from '.';
import { useFixedTableCellStyles } from '../utils';

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
  firstCell: {
    width: theme.spacing(16),
    maxWidth: theme.spacing(16),
    textOverflow: 'ellipsis',
    whiteSpace: 'nowrap',
    overflow: 'hidden',
  },
  tableRow: {
    display: 'flex',
    flexDirection: 'row',
    '&:hover': {
      cursor: 'pointer',
      backgroundColor: theme.palette.action.hover,
    },
  },
  tableCell: {
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
}));

export interface WorkcellTableProps {
  leafletMap?: LeafletContext;
  workcellContext: Record<string, DispenserResource>;
  workcells: Workcell[];
  workcellStates: Record<string, WorkcellState>;
}

interface WorkcellListRendererProps extends ListChildComponentProps {
  data: WorkcellTableProps;
  index: number;
}

export interface WorkcellRowProps {
  leafletMap?: LeafletContext;
  workcellResource: DispenserResource;
  workcell: Workcell;
  mode?: number;
  requestGuidQueue?: string[];
  secondsRemaining?: number;
}

const WorkcellRow = React.memo(
  ({
    leafletMap,
    workcell,
    workcellResource,
    mode,
    requestGuidQueue,
    secondsRemaining,
  }: WorkcellRowProps) => {
    const classes = useStyles();
    const { fixedTableCell } = useFixedTableCellStyles();
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
      <TableRow
        aria-label={`${workcell.guid}`}
        className={classes.tableRow}
        onClick={() => onWorkcellClick(workcellResource, leafletMap)}
        component="div"
      >
        {mode !== undefined && requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
          <React.Fragment>
            <TableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </TableCell>
            <TableCell
              component="div"
              variant="body"
              className={clsx(dispenserModeLabelClasses(mode), classes.tableCell, fixedTableCell)}
            >
              {dispenserModeToString(mode)}
            </TableCell>
            <TableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {requestGuidQueue.length}
            </TableCell>
            <TableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {requestGuidQueue}
            </TableCell>
            <TableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {secondsRemaining}
            </TableCell>
          </React.Fragment>
        ) : (
          <React.Fragment>
            <TableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </TableCell>
            <TableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </TableCell>
            <TableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </TableCell>
            <TableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </TableCell>
            <TableCell
              component="div"
              variant="body"
              className={clsx(classes.tableCell, fixedTableCell)}
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
  const workcellContext = data.workcellContext;
  const workcellState: WorkcellState | undefined = data.workcellStates[workcell.guid];

  return (
    <div style={style}>
      <WorkcellRow
        workcell={workcell}
        mode={workcellState?.mode}
        requestGuidQueue={workcellState?.request_guid_queue}
        secondsRemaining={workcellState?.seconds_remaining}
        workcellResource={workcellContext[workcell.guid]}
      />
    </div>
  );
};

export const WorkcellTable = ({
  workcells,
  workcellStates,
  workcellContext,
  leafletMap,
}: WorkcellTableProps): JSX.Element => {
  const classes = useStyles();
  const { fixedTableCell, fixedLastTableCell } = useFixedTableCellStyles();
  return (
    <AutoSizer disableHeight>
      {({ width }) => {
        return (
          <Table component="div" size="small" aria-label="workcell-table">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow} style={{ width: width }}>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Dispenser Name
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Op. Mode
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  No. Queued Requests
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Request Queue ID
                </TableCell>
                <TableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedLastTableCell)}
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
                  workcellContext,
                  leafletMap,
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
