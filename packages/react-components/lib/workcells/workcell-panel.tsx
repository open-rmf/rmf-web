import React from 'react';
import * as RmfModels from 'rmf-models';
import { Dispenser } from 'api-client';
import { Paper, IconButton, makeStyles, Grid, Typography, Card, Divider } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';

import { WorkcellTable } from './workcell-table';

export interface WorkcellPanelProps {
  // workcells: Dispenser[];
  dispensers: Dispenser[];
  ingestors: Dispenser[];
  workCellStates: Record<string, RmfModels.DispenserState>;
}

export interface WorkcellCellProps {
  workcell: Dispenser;
  workCellState: RmfModels.DispenserState;
}

const useStyles = makeStyles((theme) => ({
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

const WorkcellCell = (props: WorkcellCellProps): JSX.Element => {
  const { workcell, workCellState } = props;
  const classes = useStyles();

  return (
    <Paper className={classes.cellPaper}>
      {workCellState ? (
        <React.Fragment>
          <Typography align="center">{workcell.guid}</Typography>
          <Grid container direction="row">
            <Grid item xs={6}>
              <Typography
                align="center"
                variant="body2"
              >{`Queue: ${workCellState.request_guid_queue.length}`}</Typography>
            </Grid>
            <Grid item xs={6}>
              <Typography align="center" variant="body2">
                {workCellState.request_guid_queue.length
                  ? workCellState.request_guid_queue
                  : 'Unknown'}
              </Typography>
            </Grid>
          </Grid>
          <Typography align="center">{`Remaining: ${workCellState.seconds_remaining}s`}</Typography>
        </React.Fragment>
      ) : (
        <Typography color="error">{`${workcell} not sending states`}</Typography>
      )}
    </Paper>
  );
};

export function WorkcellPanel(props: WorkcellPanelProps) {
  const { dispensers, ingestors, workCellStates } = props;
  const classes = useStyles();

  const [isCellView, setIsCellView] = React.useState(true);

  return (
    <Card variant="outlined">
      <Paper className={classes.buttonBar}>
        <Grid container direction="row" justify="space-between" alignItems="center">
          <Grid item xs={6}>
            <Typography variant="h5" className={classes.panelHeader}>
              Workcell Panel
            </Typography>
          </Grid>
          <Grid item>
            <IconButton
              aria-label="view-mode"
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
              {dispensers.length > 0
                ? dispensers.map((dispenser, i) => {
                    return (
                      <Grid item xs={4} key={`${dispenser.guid}_${i}`}>
                        <WorkcellCell
                          workcell={dispenser}
                          workCellState={workCellStates[dispenser.guid]}
                        />
                      </Grid>
                    );
                  })
                : null}
            </Grid>
          </div>
          <div className={classes.cellContainer}>
            <Typography variant="h6">Ingester Table</Typography>
            <Grid container direction="row" spacing={1}>
              {ingestors.length > 0
                ? ingestors.map((ingestor, i) => {
                    return (
                      <Grid item xs={4} key={`${ingestor.guid}_${i}`}>
                        <WorkcellCell
                          workcell={ingestor}
                          workCellState={workCellStates[ingestor.guid]}
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
