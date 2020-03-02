import {
  Button,
  ButtonGroup,
  makeStyles,
} from '@material-ui/core';

import {
  Close as CloseIcon,
  KeyboardBackspace as BackIcon,
} from '@material-ui/icons';

import React from 'react';
import MainMenu, { MainMenuProps } from './main-menu';

const useStyles = makeStyles(() => ({
  container: {
    display: 'flex',
    flexFlow: 'column',
  },
}));

function handleBackClick() {
  console.log('back clicked');
}

function handleCloseClick() {
  console.log('close clicked');
}

export default function OmniPanel(props: any): JSX.Element {
  const classes = useStyles();

  const mainMenuProps: MainMenuProps = {
    onDoorsClick: () => { console.log('doors clicked'); },
    onLiftsClick: () => { console.log('lifts clicked'); },
    onRobotsClick: () => { console.log('robots clicked'); },
  };

  return (
    <div className={classes.container}>
      <ButtonGroup variant="text" fullWidth>
        <Button size="large" onClick={handleBackClick}>
          <BackIcon />
        </Button>
        <Button size="large" onClick={handleCloseClick}>
          <CloseIcon />
        </Button>
      </ButtonGroup>
      <MainMenu
        onDoorsClick={mainMenuProps.onDoorsClick}
        onLiftsClick={mainMenuProps.onLiftsClick}
        onRobotsClick={mainMenuProps.onRobotsClick}
      />
    </div>
  );
}
