import React from 'react';
import { Button, createStyles, makeStyles } from '@material-ui/core';
import { ThemeProvider } from '@material-ui/core';
import { customTheme } from './theme/theme';

const useStyles = makeStyles((theme) =>
  createStyles({
    root: {
      '&$disabled': {
        backgroundColor: theme.palette.primary.dark,
        color: theme.palette.primary.dark,
        borderRadius: 0,
      },
    },
    disabled: {},
    onClick: {
      backgroundColor: theme.palette.primary.dark,
    },
  }),
);

export interface LogoButtonProps {
  logoPath?: string;
  onClick?: () => void;
}

export const LogoButton = (props: LogoButtonProps): JSX.Element => {
  const { logoPath, onClick } = props;
  const classes = useStyles();

  if (onClick) {
    return (
      <Button disableElevation className={classes.onClick} variant="contained" onClick={onClick}>
        <img src={logoPath} style={{ width: '120px' }} alt="logo" />
      </Button>
    );
  } else {
    return (
      <ThemeProvider theme={customTheme}>
        <Button
          disableElevation
          variant="contained"
          classes={{
            root: classes.root,
            disabled: classes.disabled,
          }}
          disabled
        >
          <img src={logoPath} style={{ width: '120px' }} alt="logo" />
        </Button>
      </ThemeProvider>
    );
  }
};
