import { Table, TableBody, TableHead, TableRow, styled } from '@mui/material';
import { Dispenser } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { dispenserModeToString } from './utils';
import { useFixedTableCellStylesClasses, StyledItemTableCell } from '../utils';
import { FixedSizeList, ListChildComponentProps } from 'react-window';
import clsx from 'clsx';
import AutoSizer, { AutoSizerProps } from 'react-virtualized-auto-sizer';

const classes = {
  dispenserLabelIdle: 'workcell-dispenser-label-idle',
  dispenserLabelBusy: 'workcell-dispenser-label-busy',
  dispenserLabelOffline: 'workcell-offline-label',
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
    [`& .${classes.tableRow}`]: {
      display: 'flex',
      flexDirection: 'row',
    },
    [`& .${classes.tableCell}`]: {
      whiteSpace: 'nowrap',
      overflow: 'hidden',
      textOverflow: 'ellipsis',
    },
  }),
);

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
}

const WorkcellRow = React.memo(
  ({ workcell, mode, requestGuidQueue, secondsRemaining }: WorkcellRowProps) => {
    const { fixedTableCell } = useFixedTableCellStylesClasses;
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
      <TableRow aria-label={`${workcell.guid}`} className={classes.tableRow} component="div">
        {mode !== undefined && requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
          <React.Fragment>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </StyledItemTableCell>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(dispenserModeLabelClasses(mode), classes.tableCell, fixedTableCell)}
            >
              {dispenserModeToString(mode)}
            </StyledItemTableCell>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {requestGuidQueue.length}
            </StyledItemTableCell>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {requestGuidQueue}
            </StyledItemTableCell>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {secondsRemaining}
            </StyledItemTableCell>
          </React.Fragment>
        ) : (
          <React.Fragment>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
              title={workcell.guid}
            >
              {workcell.guid}
            </StyledItemTableCell>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </StyledItemTableCell>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </StyledItemTableCell>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </StyledItemTableCell>
            <StyledItemTableCell
              component="div"
              variant="head"
              className={clsx(classes.tableCell, fixedTableCell)}
            >
              {'NA'}
            </StyledItemTableCell>
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

  return (
    <WorkcellRow
      workcell={workcell}
      mode={workcellState?.mode}
      requestGuidQueue={workcellState?.request_guid_queue}
      secondsRemaining={workcellState?.seconds_remaining}
    />
  );
};

export const WorkcellTable = ({ workcells, workcellStates }: WorkcellTableProps): JSX.Element => {
  const { fixedTableCell } = useFixedTableCellStylesClasses;
  return (
    <StyledAutosizer disableHeight>
      {({ width }) => {
        return (
          <Table component="div" stickyHeader size="small" aria-label="workcell-table">
            <TableHead component="div">
              <TableRow component="div" className={classes.tableRow}>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Dispenser Name
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Op. Mode
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  No. Queued Requests
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Request Queue ID
                </StyledItemTableCell>
                <StyledItemTableCell
                  component="div"
                  variant="head"
                  className={clsx(classes.tableCell, fixedTableCell)}
                >
                  Seconds Remaining
                </StyledItemTableCell>
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
    </StyledAutosizer>
  );
};
