import { Divider, Grid, Typography, IconButton, styled } from '@mui/material';
import { KeyboardBackspace as BackIcon } from '@mui/icons-material';
import CloseIcon from '@mui/icons-material/Close';
import React from 'react';

interface DrawerHeaderProps {
  handleCloseButton(): void;
  handleBackButton?(): void;
  title: string;
}

const prefix = 'drawer-header';
const classes = {
  heading: `${prefix}-heading`,
  button: `${prefix}-button`,
};
const StyledDrawerHeader = styled('div')(() => ({
  [`& .${classes.heading}`]: {
    margin: '0 auto 0 calc(50% - 3rem)',
  },
  [`& .${classes.button}`]: {
    width: '3rem',
  },
}));

const DrawerHeader = (props: DrawerHeaderProps) => {
  const { handleBackButton, handleCloseButton, title } = props;

  return (
    <StyledDrawerHeader>
      {handleBackButton && (
        <Grid item>
          <IconButton
            data-testid="backDrawerButton"
            id="backDrawerButton"
            className={classes.button}
            onClick={handleBackButton}
          >
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
              data-testid="closeDrawerButton"
              id="backDrawerButton"
              className={classes.button}
              onClick={handleCloseButton}
            >
              <CloseIcon />
            </IconButton>
          </Grid>
        )}
      </Grid>
      <Divider />
    </StyledDrawerHeader>
  );
};

export default DrawerHeader;
