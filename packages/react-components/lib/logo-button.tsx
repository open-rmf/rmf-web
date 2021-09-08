import React from 'react';
import { Button, createStyles, makeStyles } from '@material-ui/core';

const useStyles = makeStyles((theme) =>
  createStyles({
    root: {
      '&$disabled': {
        backgroundColor: theme.palette.primary.main,
        color: theme.palette.primary.main,
      },
    },
    disabled: {},
    logo: {
      maxWidth: 120,
      opacity: 1,
    },
    alarmOn: {
      backgroundColor: theme.palette.error.main,
    },
    alarmOff: {
      backgroundColor: theme.palette.primary.main,
    },
  }),
);

export interface LogoButtonProps {
  logoPath?: string;
  alarm: boolean;
  onClick?: () => void;
}

export const LogoButton = (props: LogoButtonProps): JSX.Element => {
  const { alarm, logoPath, onClick } = props;
  const classes = useStyles();

  if (onClick) {
    return (
      <Button disableElevation color="primary" variant="contained" onClick={onClick}>
        <img src={logoPath} style={{ width: '120px' }} alt="logo" />
      </Button>
    );
  } else {
    return (
      <Button
        disableElevation
        variant="contained"
        classes={{
          root: classes.root,
          disabled: classes.disabled,
        }}
        className={alarm ? classes.alarmOn : classes.alarmOff}
        disabled
      >
        <img src={logoPath} style={{ width: '120px' }} alt="logo" />
      </Button>
    );
  }
};
