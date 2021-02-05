import React from 'react';
import { makeStyles, Typography, Paper } from '@material-ui/core';

export interface SpoiltItem {
  summary: string;
  errorMessage?: string;
}

export interface MainMenuSpoiltItemsProps {
  spoiltItems: SpoiltItem[];
}

const useStyles = makeStyles((theme) => ({
  paper: {
    padding: theme.spacing(1),
    margin: '1rem 0',
  },
}));

export const MainMenuSpoiltItems = (props: MainMenuSpoiltItemsProps): JSX.Element => {
  const classes = useStyles();
  const { spoiltItems } = props;

  return (
    <React.Fragment>
      <Typography color="error" variant="h6">
        Equipment Out Of Order
      </Typography>
      {spoiltItems.map((item) => {
        return (
          <Paper className={classes.paper} key={item.summary} elevation={3}>
            <Typography color="error" variant="body1">
              {item.summary}
            </Typography>
          </Paper>
        );
      })}
    </React.Fragment>
  );
};
