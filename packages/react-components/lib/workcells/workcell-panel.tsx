import { Card, Grid, IconButton, makeStyles, Paper, Typography } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import { Dispenser } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { WorkcellTable } from './workcell-table';
import { FixedSizeGrid, GridChildComponentProps } from 'react-window';

export interface WorkcellPanelProps {
  dispensers: Dispenser[];
  ingestors: Dispenser[];
  workCellStates: Record<string, RmfModels.DispenserState>;
}

export interface WorkcellDataProps extends WorkcellPanelProps {
  type: 'ingestors' | 'dispensers';
}

export interface WorkcellCellProps extends GridChildComponentProps {
  data: WorkcellDataProps;
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
  subPanelHeader: {
    marginLeft: '1rem',
  },
}));

const WorkcellCell = React.memo(
  ({ data, columnIndex, rowIndex, style }: WorkcellCellProps): JSX.Element | null => {
    let workcell: Dispenser | undefined;
    const columnCount = 3;
    let requestGuidQueue: string[] | undefined;
    let secondsRemaining: number | undefined;
    let labelId: string | undefined;
    const classes = useStyles();

    const getWorkCell = data.type === 'dispensers' ? data.dispensers : data.ingestors;
    if (rowIndex * columnCount + columnIndex <= getWorkCell.length - 1) {
      workcell = getWorkCell[rowIndex * columnCount + columnIndex];
      const state: RmfModels.DispenserState | undefined = data.workCellStates[workcell.guid];
      requestGuidQueue = state?.request_guid_queue;
      secondsRemaining = state?.seconds_remaining;
      labelId = `workcell-cell-${workcell.guid}`;
    }

    return workcell ? (
      <div style={style}>
        <Paper className={classes.cellPaper} role="region" aria-labelledby={labelId}>
          {requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
            <React.Fragment>
              <Typography id={labelId} align="center" style={{ fontWeight: 'bold' }}>
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
            <Typography id={labelId} color="error">{`${workcell} not sending states`}</Typography>
          )}
        </Paper>
      </div>
    ) : null;
  },
);

export function WorkcellPanel({
  dispensers,
  ingestors,
  workCellStates,
}: WorkcellPanelProps): JSX.Element {
  const classes = useStyles();

  const [isCellView, setIsCellView] = React.useState(true);

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
              <FixedSizeGrid
                columnCount={3}
                columnWidth={250}
                height={250}
                rowCount={Math.ceil(dispensers.length / 3)}
                rowHeight={120}
                width={760}
                itemData={{
                  dispensers,
                  ingestors,
                  workCellStates,
                  type: 'dispensers',
                }}
              >
                {WorkcellCell}
              </FixedSizeGrid>
            </Grid>
          </div>
          <div className={classes.cellContainer}>
            <Typography variant="h6">Ingester Table</Typography>
            <Grid container direction="row" spacing={1}>
              <FixedSizeGrid
                columnCount={3}
                columnWidth={250}
                height={250}
                rowCount={Math.ceil(ingestors.length / 3)}
                rowHeight={120}
                width={760}
                itemData={{
                  dispensers,
                  ingestors,
                  workCellStates,
                  type: 'ingestors',
                }}
              >
                {WorkcellCell}
              </FixedSizeGrid>
            </Grid>
          </div>
        </React.Fragment>
      ) : (
        <React.Fragment>
          {dispensers.length > 0 ? (
            <div>
              <Typography variant="h6" className={classes.subPanelHeader}>
                Dispenser Table
              </Typography>
              <WorkcellTable workcells={dispensers} workcellStates={workCellStates} />
            </div>
          ) : null}
          {ingestors.length > 0 ? (
            <div>
              <Typography variant="h6" className={classes.subPanelHeader}>
                Ingestor Table
              </Typography>
              <WorkcellTable workcells={ingestors} workcellStates={workCellStates} />
            </div>
          ) : null}
        </React.Fragment>
      )}
    </Card>
  );
}
