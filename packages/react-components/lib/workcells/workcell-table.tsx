import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import { Dispenser } from 'api-client';
import React from 'react';
import { LeafletContext } from 'react-leaflet';
import * as RmfModels from 'rmf-models';
import { dispenserModeToString, onWorkcellClick, DispenserResource } from './utils';
import { useFixedTableCellStyles } from '../utils';
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
  workcells: Dispenser[];
  workcellStates: Record<string, RmfModels.DispenserState>;
  workcellContext: Record<string, DispenserResource>;
}

interface WorkcellListRendererProps extends ListChildComponentProps {
  data: WorkcellTableProps;
  index: number;
}

export interface WorkcellRowProps {
  leafletMap?: LeafletContext;
  workcellResource: DispenserResource;
  workcell: Dispenser;
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
        onClick={() => onWorkcellClick(workcellResource, leafletMap)}
        component="div"
      >
        {mode !== undefined && requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
          <React.Fragment>
            <TableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={clsx(dispenserModeLabelClasses(mode), classes.tableCell, fixedTableCell)}
            >
              {dispenserModeToString(mode)}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {requestGuidQueue.length}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {requestGuidQueue}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {secondsRemaining}
            </TableCell>
          </React.Fragment>
        ) : (
          <React.Fragment>
            <TableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </TableCell>
            <TableCell
              component="div"
              variant="head"
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

const WorkcellListRenderer = ({ data, index }: WorkcellListRendererProps) => {
  const workcell = data.workcells[index];
  const workcellState: RmfModels.DispenserState | RmfModels.IngestorState | undefined =
    data.workcellStates[workcell.guid];
  const workcellContext = data.workcellContext;

  return (
    <WorkcellRow
      workcell={workcell}
      mode={workcellState?.mode}
      requestGuidQueue={workcellState?.request_guid_queue}
      secondsRemaining={workcellState?.seconds_remaining}
      workcellResource={workcellContext[workcell.guid]}
    />
  );
};

export const WorkcellTable = ({
  workcells,
  workcellStates,
  workcellContext,
  leafletMap,
}: WorkcellTableProps): JSX.Element => {
  const classes = useStyles();
  const { fixedTableCell } = useFixedTableCellStyles();
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
                  className={clsx(classes.tableCell, fixedTableCell)}
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
