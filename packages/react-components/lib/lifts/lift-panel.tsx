import React from 'react';
import * as RmfModels from 'rmf-models';
import {
  Paper,
  IconButton,
  makeStyles,
  Grid,
  Typography,
  Button,
  ButtonGroup,
} from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import FilterListIcon from '@material-ui/icons/FilterList';

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
  buttonGroup: {
    display: 'flex',
    justifyContent: 'center',
  },
}));

const LiftCell = (): JSX.Element => {
  const classes = useStyles();

  return (
    <Paper className={classes.cellPaper}>
      <Typography variant="body1" align="center"></Typography>
    </Paper>
  );
};

export function LiftPanel() {
  const classes = useStyles();

  const [isCellView, setIsCellView] = React.useState(true);

  return (
    <div>
      <Paper className={classes.buttonBar}>
        <IconButton className={classes.itemIcon}>
          <FilterListIcon />
        </IconButton>
        <IconButton
          aria-label="view-mode"
          className={classes.itemIcon}
          onClick={() => setIsCellView(!isCellView)}
        >
          {isCellView ? <ViewListIcon /> : <ViewModuleIcon />}
        </IconButton>
      </Paper>
      <Grid className={classes.grid} container direction="row" spacing={1}></Grid>
    </div>
  );
}
