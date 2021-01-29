import React from 'react';
import { makeStyles, Typography, Grid, Paper, Button } from '@material-ui/core';
import NavigateNextIcon from '@material-ui/icons/NavigateNext';
import { ItemSummary } from './main-menu';

export interface MainMenuItemStateProps {
  itemSummary: ItemSummary;
  handleClick: () => void;
}

const useStyles = makeStyles((theme) => ({
  paper: {
    padding: theme.spacing(0.2),
  },
  warning: {
    color: theme.palette.error.main,
    border: `2px solid ${theme.palette.error.main}`,
  },
  operational: {
    color: theme.palette.success.main,
    border: `2px solid ${theme.palette.success.main}`,
  },
  idle: {
    color: theme.palette.warning.main,
    border: `2px solid ${theme.palette.warning.main}`,
  },
  charging: {
    color: theme.palette.info.main,
    border: `2px solid ${theme.palette.info.main}`,
  },
  headerGrid: {
    display: 'flex',
    justifyContent: 'space-between',
  },
  button: {
    padding: 0,
  },
}));

export const MainMenuItemState = (props: MainMenuItemStateProps) => {
  const classes = useStyles();
  const { itemSummary, handleClick } = props;
  const getStatusLabel = (mode: string): string => {
    switch (mode) {
      case 'operational':
        return `${classes.paper} ${classes.operational}`;
      case 'outOfOrder':
        return `${classes.paper} ${classes.warning}`;
      case 'idle':
        return `${classes.paper} ${classes.idle}`;
      case 'charging':
        return `${classes.paper} ${classes.charging}`;
      default:
        return '';
    }
  };

  return (
    <React.Fragment>
      <Grid container spacing={1} direction="column">
        <Grid item className={classes.headerGrid}>
          <Typography variant="body1">{itemSummary.item}</Typography>
          <Button className={classes.button} onClick={handleClick}>
            Details <NavigateNextIcon />{' '}
          </Button>
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
                      {mode === 'outOfOrder' ? 'Out Of Order' : mode}
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
