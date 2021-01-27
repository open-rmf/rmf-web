import React from 'react';
import { makeStyles, Typography, Grid, Paper } from '@material-ui/core';
import { ItemSummary } from './main-menu';

export interface MainMenuItemStateProps {
  itemSummary: ItemSummary;
}

const useStyles = makeStyles((theme) => ({
  paper: {
    padding: theme.spacing(0.2),
    color: theme.palette.background.paper,
  },
  warning: {
    background: theme.palette.warning.light,
  },
  operational: {
    backgroundColor: theme.palette.success.light,
  },
}));

export const MainMenuItemState = (props: MainMenuItemStateProps) => {
  const classes = useStyles();
  const { itemSummary } = props;
  const getStatusLabel = (mode: string): string => {
    switch (mode) {
      case 'operational':
        return `${classes.paper} ${classes.operational}`;
      case 'broken':
        return `${classes.paper} ${classes.warning}`;
      default:
        return '';
    }
  };

  return (
    <React.Fragment>
      <Grid container spacing={1} direction="column">
        <Grid item>
          <Typography variant="h6">{itemSummary.item}</Typography>
        </Grid>
        <Grid item>
          <Grid container spacing={2} justify="flex-start" direction="row">
            {itemSummary.summary.map((summary) => {
              const mode = Object.keys(summary)[0];
              return (
                <Grid item xs={6}>
                  <Paper className={getStatusLabel(mode)} elevation={3}>
                    <Typography noWrap align="center" variant="h6">
                      {summary[mode]}
                    </Typography>
                    <Typography align="center" variant="body1">
                      {mode}
                    </Typography>
                  </Paper>
                </Grid>
              );
            })}
          </Grid>
        </Grid>
      </Grid>
    </React.Fragment>
  );
};
