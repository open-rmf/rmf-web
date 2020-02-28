import React from 'react';
import {
  Button, makeStyles, Divider, ButtonGroup,
} from '@material-ui/core';
import ArrowRightIcon from '@material-ui/icons/ArrowRight';

const borderRadius = 20;

const useStyles = makeStyles(theme => ({
  container: {
    display: 'flex',
    flexFlow: 'column',
    width: 300,
    backgroundColor: theme.palette.background.default,
    borderRadius: borderRadius,
  },

  rowContainer: {
    display: 'flex',
  },

  button: {
    flexGrow: 1,
  },

  firstRow: {
    borderTopLeftRadius: borderRadius,
    borderTopRightRadius: borderRadius,
  },

  lastRow: {
    borderBottomLeftRadius: borderRadius,
    borderBottomRightRadius: borderRadius,
  },

  alignRight: {
    textAlign: 'right',
    alignSelf: 'right',
    alignItems: 'right',
    alignContent: 'right',
    margin: 'auto',
  },
}));

export function OmniPanel() {
  const classes = useStyles();

  return (
    <div className={`${classes.container}`}>
      <ButtonGroup variant="text" fullWidth classes={{grouped: classes.firstRow}}>
        <Button>Back</Button>
        <Button>Close</Button>
      </ButtonGroup>
      <Divider flexItem />
      <Button>
        Test
      </Button>
      <ButtonGroup orientation="vertical" variant="text" fullWidth classes={{grouped: classes.lastRow}}>

        <Button>Test</Button>
      </ButtonGroup>
    </div>
  )
}

export default OmniPanel;
