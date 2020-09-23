import {
  Drawer,
  DrawerProps,
  makeStyles,
  useMediaQuery,
  Typography,
  Divider,
  IconButton,
} from '@material-ui/core';
import React from 'react';
import DrawerHeader from './help-drawer-header';
import MenuBookIcon from '@material-ui/icons/MenuBook';
import DirectionsIcon from '@material-ui/icons/Directions';
import BugReportIcon from '@material-ui/icons/BugReport';

export interface HotKeysDrawerProps extends DrawerProps {
  handleCloseButton(): void;
  setShowHotkeyDialog(): void;
}

export default function HelpDrawer(props: HotKeysDrawerProps): React.ReactElement {
  const { handleCloseButton, setShowHotkeyDialog, ...otherProps } = props;
  const classes = useStyles();
  const drawerAnchor = useMediaQuery('(max-aspect-ratio: 8/10)') ? 'bottom' : 'right';

  return (
    <Drawer PaperProps={{ className: classes.drawer }} anchor={drawerAnchor} {...otherProps}>
      <DrawerHeader handleCloseButton={handleCloseButton} title={'Help'} />
      <div className={classes.detail}>
        <div className={classes.detailLine}>
          <IconButton id="show-manual-btn" color="inherit">
            <MenuBookIcon />
          </IconButton>
          <Typography variant="h5"> Tutorial </Typography>
        </div>
        <Divider />

        <div
          className={classes.detailLine}
          onClick={() => {
            setShowHotkeyDialog();
            handleCloseButton();
          }}
        >
          <IconButton id="show-hotkeys-btn" color="inherit">
            <DirectionsIcon />
          </IconButton>
          <Typography variant="h5"> Hotkeys </Typography>
        </div>
        <Divider />

        <div className={classes.detailLine}>
          <IconButton id="show-hotkeys-btn" color="inherit">
            <BugReportIcon />
          </IconButton>
          <Typography variant="h5"> Report an error </Typography>
        </div>
        <Divider />
      </div>
    </Drawer>
  );
}

const useStyles = makeStyles((theme) => ({
  detailLine: {
    display: 'inline-flex',
    padding: theme.spacing(0.5),
    width: '100%',
    '& button': {
      padding: '0 5px 0 0',
    },
    '&:hover': {
      backgroundColor: 'rgba(0, 0, 0, 0.04)',
    },
    cursor: 'pointer',
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
}));
