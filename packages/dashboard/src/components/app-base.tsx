import { Grid, makeStyles } from '@material-ui/core';
import React from 'react';
import { loadSettings, saveSettings } from '../settings';
import {
  AppController,
  AppControllerContext,
  SettingsContext,
  Tooltips,
  TooltipsContext,
} from './app-contexts';
import { AppDrawers } from './app-drawers';
import AppBar, { AppBarProps } from './appbar';
import { RmfIngressContext } from './rmf-app';

const useStyles = makeStyles((theme) => ({
  appBase: {
    width: '100%',
    height: '100%',
    backgroundColor: theme.palette.background.default,
  },
}));

export interface AppBaseProps {
  appbarProps: AppBarProps;
}

/**
 * Contains various components that are essential to the app and provides contexts to control them.
 * Components include:
 *
 * - Settings
 * - Help
 * - Tooltip
 * - Hotkeys reference
 * - Notifications
 *
 * Also provides `AppControllerContext` to allow children components to control them.
 */
export function AppBase({
  appbarProps,
  children,
}: React.PropsWithChildren<AppBaseProps>): JSX.Element | null {
  const classes = useStyles();

  const [settings, setSettings] = React.useState(() => loadSettings());
  const [showSettings, setShowSettings] = React.useState(false);

  const [showHelp, setShowHelp] = React.useState(false);

  const [showHotkeysDialog, setShowHotkeysDialog] = React.useState(false);

  const [showTooltips, setShowTooltips] = React.useState(false);

  const { sioClient } = React.useContext(RmfIngressContext) || {};

  React.useEffect(() => {
    (async () => {
      if (!sioClient) {
        return;
      }
      // Temp workaround by hardcoding charger names
      const chargers = ['ecobot_1', 'ecobot_2', 'avidbot_1'];
      chargers.forEach((charger) => {
        sioClient?.subscribeChargerRequest(charger, (state) => {
          alert(
            `Robot ${state.charger_name} has returned for charging. Please connect its charger and press ok.`,
          );
        });
      });
    })();
  }, [sioClient]);

  const tooltips = React.useMemo<Tooltips>(
    () => ({
      showTooltips,
    }),
    [showTooltips],
  );

  const appController = React.useMemo<AppController>(
    () => ({
      showSettings: setShowSettings,
      setSettings,
      saveSettings,
      toggleSettings: () => setShowSettings((prev) => !prev),
      showHelp: setShowHelp,
      toggleHelp: () => setShowHelp((prev) => !prev),
      showHotkeysDialog: setShowHotkeysDialog,
      toggleHotkeysDialog: () => setShowHotkeysDialog((prev) => !prev),
      showTooltips: setShowTooltips,
      toggleTooltips: () => setShowTooltips((prev) => !prev),
    }),
    [],
  );

  return (
    <SettingsContext.Provider value={settings}>
      <TooltipsContext.Provider value={tooltips}>
        <AppControllerContext.Provider value={appController}>
          <Grid container direction="column" className={classes.appBase} wrap="nowrap">
            <AppBar {...appbarProps} />
            {children}
            <AppDrawers
              settings={settings}
              showHelp={showHelp}
              showHotkeysDialog={showHotkeysDialog}
              showSettings={showSettings}
            />
          </Grid>
        </AppControllerContext.Provider>
      </TooltipsContext.Provider>
    </SettingsContext.Provider>
  );
}
