import React from 'react';
import { makeStyles, Typography, Grid, Paper, Button } from '@material-ui/core';
import NavigateNextIcon from '@material-ui/icons/NavigateNext';

export interface ItemSummary<T = unknown> {
  operational: number;
  spoiltItem: T[];
}

export interface SystemSummaryItemStateProps {
  item: string;
  itemSummary: ItemSummary;
  onClick?: () => void;
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
  headerGrid: {
    display: 'flex',
    justifyContent: 'space-between',
  },
  button: {
    padding: 0,
  },
}));

export const SystemSummaryItemState = (props: SystemSummaryItemStateProps): JSX.Element => {
  const classes = useStyles();
  const { item, itemSummary, onClick } = props;

  const totalItem = itemSummary.operational + itemSummary.spoiltItem.length;
  const operationalItem = itemSummary.operational;

  const getOperationalStatusLabel = (total: number, operational: number): string => {
    if (total === operational) {
      return `${classes.paper} ${classes.operational}`;
    } else {
      return `${classes.paper} ${classes.warning}`;
    }
  };

  return (
    <Grid container spacing={1} direction="column">
      <Grid item className={classes.headerGrid}>
        <Typography variant="h6">{item}</Typography>
        <Button className={classes.button} disabled={!onClick} onClick={onClick}>
          <Typography variant="h6">Details </Typography>
          <NavigateNextIcon />
        </Button>
      </Grid>
      <Grid item>
        <Grid container justify="flex-start" direction="row">
          <Grid item xs={12}>
            <Paper className={getOperationalStatusLabel(totalItem, operationalItem)} elevation={3}>
              <Typography noWrap align="center" variant="h6">
                {`${operationalItem}/${totalItem}`}
              </Typography>
              <Typography align="center" variant="body1">
                Operational
              </Typography>
            </Paper>
          </Grid>
        </Grid>
      </Grid>
    </Grid>
  );
};
