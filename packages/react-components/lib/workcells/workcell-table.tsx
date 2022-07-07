import { styled, Table, TableBody, TableHead, TableRow } from '@mui/material';
import clsx from 'clsx';
import React from 'react';
import AutoSizer, { AutoSizerProps, Size } from 'react-virtualized-auto-sizer';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
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
const StyledAutosizer = styled((props: AutoSizerProps) => <AutoSizer {...props} />)(
  ({ theme }) => ({
    [`& .${classes.dispenserLabelIdle}`]: {
      color: theme.palette.success.main,
    },
    [`& .${classes.dispenserLabelBusy}`]: {
      color: theme.palette.error.main,
    },
    [`& .${classes.dispenserLabelOffline}`]: {
      color: theme.palette.warning.main,
    },
    [`& .${classes.tableContainer}`]: {
      maxHeight: '25vh',
    },
    [`& .${classes.firstCell}`]: {
      width: theme.spacing(16),
      maxWidth: theme.spacing(16),
      textOverflow: 'ellipsis',
      whiteSpace: 'nowrap',
      overflow: 'hidden',
    },
    [`& .${classes.tableRow}`]: {
      display: 'flex',
      flexDirection: 'row',
      '&:hover': {
        cursor: 'pointer',
        backgroundColor: theme.palette.action.hover,
      },
    },
    [`& .${classes.tableCell}`]: {
      whiteSpace: 'nowrap',
      overflow: 'hidden',
      textOverflow: 'ellipsis',
    },
  }),
);

export interface WorkcellTableProps {
  workcells: Workcell[];
  workcellStates: Record<string, WorkcellState>;
}

interface WorkcellListRendererProps extends ListChildComponentProps {
  data: WorkcellTableProps;
  index: number;
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

const WorkcellListRenderer = ({ data, index, style }: WorkcellListRendererProps) => {
  const workcell = data.workcells[index];
  const workcellState: WorkcellState | undefined = data.workcellStates[workcell.guid];

  return (
    <div style={style}>
      <WorkcellRow
        workcell={workcell}
        mode={workcellState?.mode}
        requestGuidQueue={workcellState?.request_guid_queue}
        secondsRemaining={workcellState?.seconds_remaining}
      />
    </div>
  );
};

export const WorkcellTable = ({ workcells, workcellStates }: WorkcellTableProps): JSX.Element => {
  const { fixedTableCell } = useFixedTableCellStylesClasses;
  return (
    <StyledAutosizer disableHeight>
      {({ width }: Size) => {
        return (
          <Table component="div" size="small" aria-label="workcell-table">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow}>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Dispenser Name
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Op. Mode
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  No. Queued Requests
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Request Queue ID
                </ItemTableCell>
                <ItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Seconds Remaining
                </ItemTableCell>
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
                }}
              >
                {WorkcellListRenderer}
              </FixedSizeList>
            </TableBody>
          </Table>
        );
      }}
    </StyledAutosizer>
  );
};
