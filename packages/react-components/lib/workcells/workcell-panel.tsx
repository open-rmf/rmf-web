import {
  Card,
  CardProps,
  Divider,
  Grid,
  IconButton,
  Paper,
  Typography,
  styled,
} from '@mui/material';
import ViewListIcon from '@mui/icons-material/ViewList';
import ViewModuleIcon from '@mui/icons-material/ViewModule';
import type { Dispenser, Ingestor } from 'api-client';
import React from 'react';
import { onWorkcellClick, DispenserResource } from './utils';
import AutoSizer from 'react-virtualized-auto-sizer';
import { FixedSizeGrid, GridChildComponentProps } from 'react-window';
import { Workcell, WorkcellState } from '.';
import { WorkcellTable } from './workcell-table';
import { LeafletContextInterface } from '@react-leaflet/core';

export interface WorkcellPanelProps {
  dispensers: Dispenser[];
  ingestors: Ingestor[];
  leafletMap?: LeafletContextInterface;
  workcellContext: Record<string, DispenserResource>;
  workcellStates: Record<string, WorkcellState>;
}

export interface WorkcellDataProps {
  workcells: Workcell[];
  workcellStates: Record<string, WorkcellState>;
  workcellContext: Record<string, DispenserResource>;
  leafletMap: LeafletContextInterface | undefined;
}

interface WorkcellGridData extends WorkcellDataProps {
  columnCount: number;
}

interface WorkcellGridRendererProps extends GridChildComponentProps {
  data: WorkcellGridData;
}

export interface WorkcellCellProps {
  workcell: Workcell;
  requestGuidQueue?: string[];
  secondsRemaining?: number;
  leafletMap?: LeafletContextInterface;
  workcellResource?: DispenserResource;
}

const classes = {
  container: 'workcell-panel-container',
  buttonBar: 'workcell-buttonbar',
  cellContainer: 'workcell-cell-container',
  cellPaper: 'workcell-cell-paper',
  itemIcon: 'workcell-item-icon',
  panelHeader: 'workcell-panel-header',
  subPanelHeader: 'workcell-sub-panel-header',
  tableDiv: 'workcell-table-div',
  nameField: 'workcell-name-field',
  grid: 'workcell-grid',
};
const StyledCard = styled((props: CardProps) => <Card {...props} />)(({ theme }) => ({
  [`&.${classes.container}`]: {
    margin: theme.spacing(1),
  },
  [`& .${classes.buttonBar}`]: {
    display: 'flex',
    justifyContent: 'flex-end',
    borderRadius: 0,
    backgroundColor: theme.palette.primary.main,
  },
  [`& .${classes.cellContainer}`]: {
    padding: theme.spacing(1),
    maxHeight: '25vh',
    margin: theme.spacing(1),
    overflowY: 'auto',
    overflowX: 'hidden',
  },
  [`& .${classes.cellPaper}`]: {
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
  },
  [`& .${classes.grid}`]: {
    padding: theme.spacing(2),
    paddingTop: theme.spacing(1),
  },
  [`& .${classes.itemIcon}`]: {
    color: theme.palette.primary.contrastText,
  },
  [`& .${classes.panelHeader}`]: {
    color: theme.palette.primary.contrastText,
    marginLeft: theme.spacing(2),
  },
  [`& .${classes.subPanelHeader}`]: {
    marginLeft: theme.spacing(2),
    color: theme.palette.primary.contrastText,
  },
  [`& .${classes.tableDiv}`]: {
    margin: theme.spacing(1),
    padding: theme.spacing(1),
  },
  [`& .${classes.nameField}`]: {
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
    workcell,
    requestGuidQueue,
    secondsRemaining,
    leafletMap,
    workcellResource,
  }: WorkcellCellProps): JSX.Element | null => {
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
  let leafletMap: LeafletContextInterface | undefined;
  const columnCount = data.columnCount;
  const { workcells, workcellStates, workcellContext } = data;

  if (rowIndex * columnCount + columnIndex <= workcells.length - 1) {
    workcell = workcells[rowIndex * columnCount + columnIndex];
    workcellState = workcellStates[workcell.guid];
    workcellResource = workcellContext[workcell.guid];
    leafletMap = data.leafletMap;
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
  const [isCellView, setIsCellView] = React.useState(true);
  const columnWidth = 250;

  return (
    <StyledCard variant="outlined" className={classes.container}>
      <Paper className={classes.buttonBar}>
        <Grid container direction="row" justifyContent="space-between" alignItems="center">
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
                leafletMap={leafletMap}
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
                leafletMap={leafletMap}
              />
            </div>
          ) : null}
        </React.Fragment>
      )}
    </StyledCard>
  );
}
