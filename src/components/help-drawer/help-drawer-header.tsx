import {
  Divider,
  Drawer,
  DrawerProps,
  makeStyles,
  useMediaQuery,
  Grid,
  Typography,
  IconButton,
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import { KeyboardBackspace as BackIcon } from '@material-ui/icons';
import React from 'react';

interface DrawerHeaderProps {
  handleCloseButton(): void;
  handleBackButton?(): void;
  title: string;
}

const DrawerHeader = (props: DrawerHeaderProps) => {
  const { handleBackButton, handleCloseButton, title } = props;
  const classes = useStyles();

  return (
    <>
      {handleBackButton && (
        <Grid item>
          <IconButton id="backDrawerButton" className={classes.button} onClick={handleBackButton}>
            <BackIcon />
          </IconButton>
        </Grid>
      )}
      <Grid container alignItems="center">
        <Grid item className={classes.heading}>
          <Typography variant="h6">{title}</Typography>
        </Grid>

        {handleCloseButton && (
          <Grid item>
            <IconButton
              id="closeDrawerButton"
              className={classes.button}
              onClick={handleCloseButton}
            >
              <CloseIcon />
            </IconButton>
          </Grid>
        )}
      </Grid>
      <Divider />
    </>
  );
};

export default DrawerHeader;

const useStyles = makeStyles((theme) => ({
  detailLine: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
    width: '100%',
  },
  detail: {
    display: 'flex',
    flexFlow: 'column',
    padding: '1rem',
  },
  drawer: {
    '@media (min-aspect-ratio: 8/10)': {
      width: 300,
    },
    '@media (max-aspect-ratio: 8/10)': {
      width: '100%',
    },
  },
  heading: {
    margin: '0 auto 0 calc(50% - 3rem)',
  },
  button: {
    width: '3rem',
  },
}));
