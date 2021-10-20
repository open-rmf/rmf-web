import { Grid, makeStyles, Paper, Typography } from '@material-ui/core';
import React from 'react';

export interface WorkcellCellProps {
  guid: string;
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
  },
  panelHeader: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
    marginLeft: '1rem',
  },
  subPanelHeader: {
    marginLeft: '1rem',
  },
}));

export const WorkcellCell: React.FC<WorkcellCellProps> = ({
  guid,
  requestGuidQueue,
  secondsRemaining,
}) => {
  const classes = useStyles();

  const labelId = `workcell-cell-${guid}`;

  return (
    <Paper className={classes.cellPaper} role="region" aria-labelledby={labelId}>
      {requestGuidQueue !== undefined && secondsRemaining !== undefined ? (
        <React.Fragment>
          <Typography id={labelId} align="center" style={{ fontWeight: 'bold' }}>
            {guid}
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
        <Typography id={labelId} color="error">{`${guid} not sending states`}</Typography>
      )}
    </Paper>
  );
};
