import {
  Button,
  ButtonGroup,
  makeStyles,
  Typography,
} from '@material-ui/core';

import React from 'react';

const useStyles = makeStyles(() => ({
  button: {
    justifyContent: 'left',
  },
}));

export interface MainMenuProps {
  onDoorsClick?: (event: React.MouseEvent<HTMLButtonElement, MouseEvent>) => void;
  onLiftsClick?: (event: React.MouseEvent<HTMLButtonElement, MouseEvent>) => void;
  onRobotsClick?: (event: React.MouseEvent<HTMLButtonElement, MouseEvent>) => void;
}

export default function MainMenu(props: MainMenuProps): JSX.Element {
  const classes = useStyles();

  return (
    <ButtonGroup orientation="vertical" variant="text" fullWidth>
      <Button size="large" className={classes.button} onClick={props.onDoorsClick}>
        <Typography variant="h5">Doors</Typography>
      </Button>
      <Button size="large" className={classes.button} onClick={props.onLiftsClick}>
        <Typography variant="h5">Lifts</Typography>
      </Button>
      <Button size="large" className={classes.button} onClick={props.onRobotsClick}>
        <Typography variant="h5">Robots</Typography>
      </Button>
    </ButtonGroup>
  );
}
