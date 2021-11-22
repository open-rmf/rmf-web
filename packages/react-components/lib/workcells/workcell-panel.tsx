import { Card, Divider, Grid, IconButton, makeStyles, Paper, Typography } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import type { Dispenser, Ingestor } from 'api-client';
import React from 'react';
import { LeafletContext } from 'react-leaflet';
import { onWorkcellClick, DispenserResource } from './utils';
import AutoSizer from 'react-virtualized-auto-sizer';
import { FixedSizeGrid, GridChildComponentProps } from 'react-window';
import { Workcell, WorkcellState } from '.';
import { WorkcellTable } from './workcell-table';

export interface WorkcellPanelProps {
  dispensers: Dispenser[];
  leafletMap?: LeafletContext;
  workcellContext: Record<string, DispenserResource>;
  ingestors: Ingestor[];
  workcellStates: Record<string, WorkcellState>;
}

export interface WorkcellDataProps {
  workcells: Workcell[];
  workcellStates: Record<string, WorkcellState>;
}

interface WorkcellGridData extends WorkcellDataProps {
  columnCount: number;
}

interface WorkcellGridRendererProps extends GridChildComponentProps {
  data: WorkcellGridData;
}

export interface WorkcellCellProps {
  workcellResource?: DispenserResource;
  leafletMap?: LeafletContext;
  workcell: Workcell;
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
    borderRadius: 0,
    backgroundColor: theme.palette.primary.main,
  },
  cellContainer: {
    padding: theme.spacing(1),
    maxHeight: '25vh',
    margin: theme.spacing(1),
    overflowY: 'auto',
    overflowX: 'hidden',
  },
  cellPaper: {
    padding: theme.spacing(1),
    backgroundColor: theme.palette.background.paper,
    border: 1,
    borderStyle: 'solid',
    borderColor: theme.palette.primary.main,
    '&:hover': {
      cursor: 'pointer',
      backgroundColor: theme.palette.action.hover,
    },
    margin: theme.spacing(1),
    height: '60%',
  },
  grid: {
    padding: theme.spacing(2),
    paddingTop: theme.spacing(1),
  },
  itemIcon: {
    color: theme.palette.primary.contrastText,
  },
  panelHeader: {
    color: theme.palette.primary.contrastText,
    marginLeft: theme.spacing(2),
  },
  subPanelHeader: {
    marginLeft: theme.spacing(2),
    color: theme.palette.primary.contrastText,
  },
  tableDiv: {
    margin: theme.spacing(1),
    padding: theme.spacing(1),
  },
  nameField: {
    fontWeight: 'bold',
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
  bottomTable: {
    marginTop: theme.spacing(2),
  },
}));

const WorkcellCell = React.memo(
  ({
    workcellResource,
    leafletMap,
    workcell,
    requestGuidQueue,
    secondsRemaining,
  }: WorkcellCellProps): JSX.Element => {
    const classes = useStyles();
    const labelId = `workcell-cell-${workcell.guid}`;

    return (
      <Paper
        className={classes.cellPaper}
        role="region"
        aria-labelledby={labelId}
        onClick={() => workcellResource && onWorkcellClick(workcellResource, leafletMap)}
      >
        {requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
          <React.Fragment>
            <Typography
              noWrap
              id={labelId}
              align="center"
              className={classes.nameField}
              title={workcell?.guid}
            >
              {workcell?.guid}
            </Typography>
            <Grid container direction="row">
              <Grid item xs={8}>
                <Typography
                  align="center"
                  variant="body2"
                >{`Queue: ${requestGuidQueue.length}`}</Typography>
              </Grid>
              <Grid item xs={4}>
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

const WorkcellGridRenderer = ({
  data,
  columnIndex,
  rowIndex,
  style,
}: WorkcellGridRendererProps) => {
  let workcell: Workcell | undefined;
  let workcellState: WorkcellState | undefined;
  let workcellResource: DispenserResource | undefined;
  let leafletMap: LeafletContext | undefined;
  const columnCount = data.columnCount;
  const { workcells, workcellStates } = data;

  if (rowIndex * columnCount + columnIndex <= workcells.length - 1) {
    workcell = workcells[rowIndex * columnCount + columnIndex];
    workcellState = workcellStates[workcell.guid];
  }

  return workcell ? (
    <div style={style}>
      <WorkcellCell
        workcell={workcell}
        requestGuidQueue={workcellState?.request_guid_queue}
        secondsRemaining={workcellState?.seconds_remaining}
        workcellResource={workcellResource}
        leafletMap={leafletMap}
      />
    </div>
  ) : null;
};

export function WorkcellPanel({
  dispensers,
  leafletMap,
  ingestors,
  workcellContext,
  workcellStates,
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
            <Typography variant="h6">Dispensers</Typography>
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
                        workcells: dispensers,
                        workcellStates,
                        type: 'dispensers',
                        workcellContext,
                        leafletMap,
                      }}
                    >
                      {WorkcellGridRenderer}
                    </FixedSizeGrid>
                  );
                }}
              </AutoSizer>
            </Grid>
          </div>
          <Divider />
          <div className={classes.cellContainer}>
            <Typography variant="h6">Ingestors</Typography>
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
                        workcells: ingestors,
                        workcellStates,
                        type: 'ingestors',
                        workcellContext,
                        leafletMap,
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
              <WorkcellTable
                workcells={dispensers}
                workcellStates={workcellStates}
                workcellContext={workcellContext}
              />
            </div>
          ) : null}
          {ingestors.length > 0 ? (
            <div className={classes.tableDiv}>
              <Typography variant="h6">Ingestor Table</Typography>
              <WorkcellTable
                workcells={ingestors}
                workcellStates={workcellStates}
                workcellContext={workcellContext}
              />
            </div>
          ) : null}
        </React.Fragment>
      )}
    </Card>
  );
}
