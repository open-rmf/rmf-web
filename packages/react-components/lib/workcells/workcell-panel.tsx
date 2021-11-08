import { Card, Grid, IconButton, makeStyles, Paper, Typography } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import { Dispenser } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { WorkcellTable } from './workcell-table';
import { FixedSizeGrid, GridChildComponentProps } from 'react-window';
import AutoSizer from 'react-virtualized-auto-sizer';

export interface WorkcellPanelProps {
  dispensers: Dispenser[];
  ingestors: Dispenser[];
  workCellStates: Record<string, RmfModels.DispenserState>;
}

export interface WorkcellDataProps extends WorkcellPanelProps {
  type: 'ingestors' | 'dispensers';
}

interface WorkcellGridData extends WorkcellDataProps {
  columnCount: number;
}

interface WorkcellGridRendererProps extends GridChildComponentProps {
  data: WorkcellGridData;
}

export interface WorkcellCellProps {
  workcell: Dispenser;
  requestGuidQueue?: string[];
  secondsRemaining?: number;
}

const useStyles = makeStyles((theme) => ({
  container: {
    margin: theme.spacing(1),
  },
  buttonBar: {
    display: 'flex',
    justifyContent: 'flex-end',
    borderRadius: '0px',
    backgroundColor: theme.palette.primary.main,
  },
  cellContainer: {
    paddingLeft: '1rem',
    paddingRight: '1rem',
    paddingBottom: '1rem',
  },
  cellPaper: {
    padding: '0.5rem',
    backgroundColor: theme.palette.info.light,
    margin: '0.5rem',
    height: '84px',
  },
  itemIcon: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
  },
  panelHeader: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
    marginLeft: '1rem',
  },
  tableDiv: {
    margin: '0 1rem',
  },
  nameField: {
    fontWeight: 'bold',
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
}));

const WorkcellCell = React.memo(
  ({ workcell, requestGuidQueue, secondsRemaining }: WorkcellCellProps): JSX.Element | null => {
    const labelId = `workcell-cell-${workcell.guid}`;
    const classes = useStyles();

    return (
      <Paper className={classes.cellPaper} role="region" aria-labelledby={labelId}>
        {requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
          <React.Fragment>
            <Typography
              id={labelId}
              align="center"
              className={classes.nameField}
              title={workcell?.guid}
            >
              {workcell?.guid}
            </Typography>
            <Grid container direction="row">
              <Grid item xs={6}>
                <Typography
                  align="center"
                  variant="body2"
                >{`Queue: ${requestGuidQueue.length}`}</Typography>
              </Grid>
              <Grid item xs={6}>
                <Typography align="center" variant="body2">
                  {requestGuidQueue.length}
                </Typography>
              </Grid>
            </Grid>
            <Typography align="center">{`Remaining: ${secondsRemaining}s`}</Typography>
          </React.Fragment>
        ) : (
          <Typography
            id={labelId}
            color="error"
          >{`${workcell.guid} not sending states`}</Typography>
        )}
      </Paper>
    );
  },
);

const WorkcellGridRenderer = ({ data, columnIndex, rowIndex }: WorkcellGridRendererProps) => {
  let workcell: Dispenser | undefined;
  let workcellState: RmfModels.DispenserState | undefined;
  const columnCount = data.columnCount;
  const getWorkCell = data.type === 'dispensers' ? data.dispensers : data.ingestors;

  if (rowIndex * columnCount + columnIndex <= getWorkCell.length - 1) {
    workcell = getWorkCell[rowIndex * columnCount + columnIndex];
    workcellState = data.workCellStates[workcell.guid];
  }

  return workcell ? (
    <div>
      <WorkcellCell
        workcell={workcell}
        requestGuidQueue={workcellState?.request_guid_queue}
        secondsRemaining={workcellState?.seconds_remaining}
      />
    </div>
  ) : null;
};

export function WorkcellPanel({
  dispensers,
  ingestors,
  workCellStates,
}: WorkcellPanelProps): JSX.Element {
  const classes = useStyles();
  const [isCellView, setIsCellView] = React.useState(true);
  const columnWidth = 250;

  return (
    <Card variant="outlined" className={classes.container}>
      <Paper className={classes.buttonBar}>
        <Grid container direction="row" justify="space-between" alignItems="center">
          <Grid item xs={6}>
            <Typography variant="h5" className={classes.panelHeader}>
              Workcells
            </Typography>
          </Grid>
          <Grid item>
            <IconButton
              aria-label="view mode"
              className={classes.itemIcon}
              onClick={() => setIsCellView(!isCellView)}
            >
              {isCellView ? <ViewListIcon /> : <ViewModuleIcon />}
            </IconButton>
          </Grid>
        </Grid>
      </Paper>
      {isCellView ? (
        <React.Fragment>
          <div className={classes.cellContainer}>
            <Typography variant="h6">Dispenser Table</Typography>
            <Grid container direction="row" spacing={1}>
              <AutoSizer disableHeight>
                {({ width }) => {
                  const columnCount = Math.floor(width / columnWidth);
                  return (
                    <FixedSizeGrid
                      columnCount={columnCount}
                      columnWidth={columnWidth}
                      height={250}
                      rowCount={Math.ceil(dispensers.length / columnCount)}
                      rowHeight={120}
                      width={width}
                      itemData={{
                        columnCount,
                        dispensers,
                        ingestors,
                        workCellStates,
                        type: 'dispensers',
                      }}
                    >
                      {WorkcellGridRenderer}
                    </FixedSizeGrid>
                  );
                }}
              </AutoSizer>
            </Grid>
          </div>
          <div className={classes.cellContainer}>
            <Typography variant="h6">Ingester Table</Typography>
            <Grid container direction="row" spacing={1}>
              <AutoSizer disableHeight>
                {({ width }) => {
                  const columnCount = Math.floor(width / columnWidth);
                  return (
                    <FixedSizeGrid
                      columnCount={columnCount}
                      columnWidth={columnWidth}
                      height={250}
                      rowCount={Math.ceil(ingestors.length / columnCount)}
                      rowHeight={120}
                      width={width}
                      itemData={{
                        columnCount,
                        dispensers,
                        ingestors,
                        workCellStates,
                        type: 'ingestors',
                      }}
                    >
                      {WorkcellGridRenderer}
                    </FixedSizeGrid>
                  );
                }}
              </AutoSizer>
            </Grid>
          </div>
        </React.Fragment>
      ) : (
        <React.Fragment>
          {dispensers.length > 0 ? (
            <div className={classes.tableDiv}>
              <Typography variant="h6">Dispenser Table</Typography>
              <WorkcellTable workcells={dispensers} workcellStates={workCellStates} />
            </div>
          ) : null}
          {ingestors.length > 0 ? (
            <div className={classes.tableDiv}>
              <Typography variant="h6">Ingestor Table</Typography>
              <WorkcellTable workcells={ingestors} workcellStates={workCellStates} />
            </div>
          ) : null}
        </React.Fragment>
      )}
    </Card>
  );
}
