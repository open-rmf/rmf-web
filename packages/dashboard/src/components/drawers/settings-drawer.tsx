import {
  Drawer,
  DrawerProps,
  Grid,
  IconButton,
  Typography,
  useMediaQuery,
  styled,
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import React from 'react';
import { Settings } from '../../settings';

export interface SettingsDrawerProps extends DrawerProps {
  settings: Readonly<Settings>;
  onSettingsChange(settings: Settings): void;
  handleCloseButton: React.Dispatch<React.SetStateAction<boolean>>;
}

const classes = {
  drawer: 'settings-drawer-paper',
  legendLabel: 'settings-drawer-legend-label',
  trajGroup: 'settings-drawer-traj-group',
  flexBasis: 'settings-drawer-flex-basis',
  heading: 'settings-drawer-heading',
  button: 'settings-drawer-button',
};
const SettingsDrawerRoot = styled((props: DrawerProps) => <Drawer {...props} />)(({ theme }) => ({
  [`& .${classes.drawer}`]: {
    '@media (min-aspect-ratio: 8/10)': {
      width: 300,
    },
    '@media (max-aspect-ratio: 8/10)': {
      width: '100%',
    },
  },
  [`& .${classes.legendLabel}`]: {
    '@media (min-aspect-ratio: 8/10)': {
      fontSize: theme.typography.h6.fontSize,
      paddingBottom: theme.spacing(1),
      textAlign: 'center',
    },
    '@media (max-aspect-ratio: 8/10)': {
      fontSize: theme.typography.h6.fontSize,
      padding: theme.spacing(1),
      textAlign: 'center',
    },
  },
  [`& .${classes.trajGroup}`]: {
    '@media (min-aspect-ratio: 8/10)': {
      flexDirection: 'row',
      paddingLeft: theme.spacing(8),
      margin: '1rem 0',
    },
    '@media (max-aspect-ratio: 8/10)': {
      flexDirection: 'row',
      paddingLeft: theme.spacing(8),
    },
  },
  [`& .${classes.flexBasis}`]: {
    flexBasis: '40%',
  },
  [`& .${classes.heading}`]: {
    margin: '0 auto 0 calc(50% - 3rem)',
  },
  [`& .${classes.button}`]: {
    width: '3rem',
  },
}));

// Drawer is empty because there is no settings.
export default function SettingsDrawer(props: SettingsDrawerProps): React.ReactElement {
  const { settings, onSettingsChange, handleCloseButton, ...otherProps } = props;

  const drawerAnchor = useMediaQuery('(max-aspect-ratio: 8/10') ? 'bottom' : 'right';

  const modalProp = {
    disableEnforceFocus: true,
  };

  return (
    <SettingsDrawerRoot
      PaperProps={{ className: classes.drawer }}
      anchor={drawerAnchor}
      ModalProps={modalProp}
      {...otherProps}
    >
      <Grid container alignItems="center">
        <Grid item className={classes.heading}>
          <Typography variant="h6">Settings</Typography>
        </Grid>
        <Grid item>
          <IconButton
            id="closeDrawerButton"
            aria-label="close-settings"
            className={classes.button}
            onClick={() => handleCloseButton(false)}
          >
            <CloseIcon />
          </IconButton>
        </Grid>
      </Grid>
    </SettingsDrawerRoot>
  );
}
