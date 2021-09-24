import {
  Divider,
  Drawer,
  DrawerProps,
  FormGroup,
  FormLabel,
  FormControlLabel,
  Grid,
  IconButton,
  makeStyles,
  Switch,
  Typography,
  useMediaQuery,
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import React from 'react';
import { Settings, ThemeMode } from '../../settings';

export interface SettingsDrawerProps extends DrawerProps {
  settings: Readonly<Settings>;
  onSettingsChange(settings: Settings): void;
  handleCloseButton: React.Dispatch<React.SetStateAction<boolean>>;
}

// Drawer is empty because there is no settings.
export default function SettingsDrawer(props: SettingsDrawerProps): React.ReactElement {
  const classes = useStyles();
  const { settings, onSettingsChange, handleCloseButton, ...otherProps } = props;
  const { themeMode } = settings;

  const themeText = React.useMemo(
    () => Object.keys(ThemeMode).slice(Object.keys(ThemeMode).length * 0.5),
    [],
  );

  const drawerAnchor = useMediaQuery('(max-aspect-ratio: 8/10') ? 'bottom' : 'right';

  const modalProp = {
    disableEnforceFocus: true,
  };

  function handleThemeModeChange(ev: React.ChangeEvent<HTMLInputElement>): void {
    const newSettings: Settings = { ...settings, themeMode: Number(ev.target.checked) };
    onSettingsChange && onSettingsChange(newSettings);
  }

  return (
    <Drawer
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
      <FormGroup className={classes.formGroup}>
        <FormLabel component="legend" className={classes.legendLabel}>
          Theme Mode
        </FormLabel>
        <FormControlLabel
          className={classes.swtichButton}
          control={
            <Switch
              onChange={handleThemeModeChange}
              name={'theme switch'}
              checked={themeMode === ThemeMode.Dark}
            />
          }
          label={themeText[settings.themeMode]}
        />
        <Divider />
      </FormGroup>
    </Drawer>
  );
}

const useStyles = makeStyles((theme) => ({
  drawer: {
    '@media (min-aspect-ratio: 8/10)': {
      width: 300,
    },
    '@media (max-aspect-ratio: 8/10)': {
      width: '100%',
    },
  },
  legendLabel: {
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
  trajGroup: {
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
  swtichButton: {
    margin: '0 auto',
    marginBottom: '1rem',
  },
  flexBasis: {
    flexBasis: '40%',
  },
  heading: {
    margin: '0 auto 0 calc(50% - 3rem)',
  },
  button: {
    width: '3rem',
  },
  formGroup: {
    marginTop: '1rem',
  },
}));
