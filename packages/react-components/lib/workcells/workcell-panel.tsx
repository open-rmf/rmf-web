import { Card, Grid, IconButton, makeStyles, Paper, Typography } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import { Dispenser } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { WorkcellTable } from './workcell-table';

export interface WorkcellPanelProps {
  dispensers: Dispenser[];
  ingestors: Dispenser[];
  workCellStates: Record<string, RmfModels.DispenserState>;
}

export interface WorkcellCellProps {
  workcell: Dispenser;
  requestGuidQueue?: string[];
  secondsRemaining?: number;
}

const useStyles = makeStyles((theme) => ({
  buttonBar: {
    display: 'flex',
    justifyContent: 'flex-end',
    borderRadius: 0,
    backgroundColor: theme.palette.primary.light,
  },
  cellContainer: {
    padding: theme.spacing(2),
    paddingTop: theme.spacing(1),
    backgroundColor: theme.palette.primary.main,
    color: theme.palette.getContrastText(theme.palette.primary.main),
  },
  cellPaper: {
    padding: theme.spacing(1),
    backgroundColor: theme.palette.primary.light,
    color: theme.palette.getContrastText(theme.palette.primary.light),
  },
  itemIcon: {
    color: theme.palette.getContrastText(theme.palette.primary.light),
  },
  panelHeader: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
    marginLeft: theme.spacing(2),
  },
  subPanelHeader: {
    marginLeft: theme.spacing(2),
    color: theme.palette.getContrastText(theme.palette.primary.light),
  },
}));

const WorkcellCell = React.memo(
  ({ workcell, requestGuidQueue, secondsRemaining }: WorkcellCellProps): JSX.Element => {
    const classes = useStyles();

    const labelId = `workcell-cell-${workcell.guid}`;

    return (
      <Paper className={classes.cellPaper} role="region" aria-labelledby={labelId}>
        {requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
          <React.Fragment>
            <Typography id={labelId} align="center" style={{ fontWeight: 'bold' }}>
              {workcell.guid}
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
    );
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
    <Card variant="outlined">
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
              {dispensers.length > 0
                ? dispensers.map((dispenser, i) => {
                    const state: RmfModels.DispenserState | undefined =
                      workCellStates[dispenser.guid];
                    return (
                      <Grid item xs={4} key={`${dispenser.guid}_${i}`}>
                        <WorkcellCell
                          workcell={dispenser}
                          requestGuidQueue={state?.request_guid_queue}
                          secondsRemaining={state?.seconds_remaining}
                        />
                      </Grid>
                    );
                  })
                : null}
            </Grid>
          </div>
          <div className={classes.cellContainer}>
            <Typography variant="h6">Ingestors</Typography>
            <Grid container direction="row" spacing={1}>
              {ingestors.length > 0
                ? ingestors.map((ingestor, i) => {
                    const state: RmfModels.IngestorState | undefined =
                      workCellStates[ingestor.guid];
                    return (
                      <Grid item xs={4} key={`${ingestor.guid}_${i}`}>
                        <WorkcellCell
                          workcell={ingestor}
                          requestGuidQueue={state?.request_guid_queue}
                          secondsRemaining={state?.seconds_remaining}
                        />
                      </Grid>
                    );
                  })
                : null}
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
