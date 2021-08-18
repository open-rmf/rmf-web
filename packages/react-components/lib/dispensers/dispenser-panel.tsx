import React from 'react';
import * as RmfModels from 'rmf-models';
import { Dispenser } from 'api-client';
import { Paper, IconButton, makeStyles, Grid, Typography } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';

import { DispenserTable } from './dispenser-table';

export interface DispenserPanelProps {
  dispensers: Dispenser[];
  dispenserStates: Record<string, RmfModels.DispenserState>;
}

export interface DispenserCellProps {
  dispenser: Dispenser;
  dispenserState: RmfModels.DispenserState;
}

const useStyles = makeStyles((theme) => ({
  buttonBar: {
    display: 'flex',
    justifyContent: 'flex-end',
    borderRadius: '0px',
    backgroundColor: theme.palette.primary.main,
  },
  grid: {
    padding: '1rem',
  },
  cellPaper: {
    padding: '0.5rem',
    backgroundColor: theme.palette.info.light,
  },
  itemIcon: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
  },
}));

const DispenserCell = (props: DispenserCellProps): JSX.Element => {
  const { dispenser, dispenserState } = props;
  const classes = useStyles();

  return (
    <Paper className={classes.cellPaper}>
      {dispenserState ? (
        <React.Fragment>
          <Typography align="center">{dispenser.guid}</Typography>
          <Grid container direction="row">
            <Grid item xs={6}>
              <Typography
                align="center"
                variant="body2"
              >{`Queue: ${dispenserState.request_guid_queue.length}`}</Typography>
            </Grid>
            <Grid item xs={6}>
              <Typography align="center" variant="body2">
                {dispenserState.request_guid_queue.length
                  ? dispenserState.request_guid_queue
                  : 'Unknown'}
              </Typography>
            </Grid>
          </Grid>
          <Typography align="center">{`Remaining: ${dispenserState.seconds_remaining}s`}</Typography>
        </React.Fragment>
      ) : (
        <Typography color="error">{`${dispenser} not sending states`}</Typography>
      )}
    </Paper>
  );
};

export function DispenserPanel(props: DispenserPanelProps) {
  const { dispensers, dispenserStates } = props;
  const classes = useStyles();

  const [isCellView, setIsCellView] = React.useState(true);

  return (
    <div>
      <Paper className={classes.buttonBar}>
        <IconButton
          aria-label="view-mode"
          className={classes.itemIcon}
          onClick={() => setIsCellView(!isCellView)}
        >
          {isCellView ? <ViewListIcon /> : <ViewModuleIcon />}
        </IconButton>
      </Paper>
      <Grid className={classes.grid} container direction="row" spacing={1}>
        {isCellView ? (
          dispensers.map((dispenser, i) => {
            return (
              <Grid item xs={4} key={`${dispenser.guid}_${i}`}>
                <DispenserCell
                  dispenser={dispenser}
                  dispenserState={dispenserStates[dispenser.guid]}
                />
              </Grid>
            );
          })
        ) : (
          <DispenserTable dispensers={dispensers} dispenserStates={dispenserStates} />
        )}
      </Grid>
    </div>
  );
}
