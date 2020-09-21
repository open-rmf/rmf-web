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
import React from 'react';
import CloseIcon from '@material-ui/icons/Close';
import { getApplicationKeyMap } from 'react-hotkeys';

export interface HotKeysDrawerProps extends DrawerProps {
  handleCloseButton(): void;
}

export default function HotKeysDrawer(props: HotKeysDrawerProps): React.ReactElement {
  const { handleCloseButton, ...otherProps } = props;
  const classes = useStyles();
  const drawerAnchor = useMediaQuery('(max-aspect-ratio: 8/10)') ? 'bottom' : 'right';
  const keyMap = getApplicationKeyMap();
  return (
    <Drawer PaperProps={{ className: classes.drawer }} anchor={drawerAnchor} {...otherProps}>
      <Grid container alignItems="center">
        <Grid item className={classes.heading}>
          <Typography variant="h6">HotKeys</Typography>
        </Grid>
        <Grid item>
          <IconButton id="closeDrawerButton" className={classes.button} onClick={handleCloseButton}>
            <CloseIcon />
          </IconButton>
        </Grid>
      </Grid>

      <Divider />
      <div className={classes.detail}>
        {Object.values(keyMap).map((hotkey) => {
          return (
            hotkey.name && (
              <>
                <div key={hotkey.name} className={classes.detailLine}>
                  <Typography variant="body1">{hotkey.name}:</Typography>
                  <Typography variant="body1">{hotkey.sequences[0].sequence}</Typography>
                </div>
                <Divider />
              </>
            )
          );
        })}
      </div>
    </Drawer>
  );
}

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
