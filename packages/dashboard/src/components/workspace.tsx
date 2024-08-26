import AddIcon from '@mui/icons-material/Add';
import DesignModeIcon from '@mui/icons-material/AutoFixNormal';
import { Box, Fab, IconButton, Menu, MenuItem, Typography, useTheme } from '@mui/material';
import React from 'react';
import { WindowContainer, WindowLayout } from 'react-components';

import { useAppController } from '../hooks/use-app-controller';
import { MicroAppManifest } from './micro-app';

export interface InitialWindow {
  layout: Omit<WindowLayout, 'i'>;
  microApp: MicroAppManifest;
}

export interface WorkspaceProps {
  /**
   * Initial windows in the workspace, this only affects the initial list of windows, changing it
   * will not affect the current state of the workspace.
   */
  initialWindows: InitialWindow[];

  /**
   * Whether to allow customizing the workspace.
   */
  allowDesignMode?: boolean;

  /**
   * List of micro apps available when customizing the workspace.
   */
  appRegistry?: MicroAppManifest[];

  onLayoutChange?: (windows: { layout: WindowLayout; microApp: MicroAppManifest }[]) => void;
}

export const Workspace = React.memo(
  ({
    initialWindows,
    allowDesignMode = false,
    appRegistry = [],
    onLayoutChange,
  }: WorkspaceProps) => {
    const theme = useTheme();
    const appController = useAppController();
    const windowId = React.useRef(0);
    const windowApps = React.useRef<Record<string, MicroAppManifest>>({});
    const [layout, setLayout] = React.useState(() =>
      initialWindows.map<WindowLayout>((w) => {
        const l = { i: `window-${windowId.current}`, ...w.layout };
        windowApps.current[l.i] = w.microApp;
        ++windowId.current;
        return l;
      }),
    );
    const [designMode, setDesignMode] = React.useState(false);
    const [addMenuAnchor, setAddMenuAnchor] = React.useState<HTMLElement | null>(null);

    React.useEffect(() => {
      if (!allowDesignMode) {
        return;
      }

      appController.setExtraAppbarItems(
        <IconButton
          color="inherit"
          sx={{ opacity: designMode ? undefined : theme.palette.action.disabledOpacity }}
          onClick={() => setDesignMode((prev) => !prev)}
        >
          <DesignModeIcon />
        </IconButton>,
      );
      return () => appController.setExtraAppbarItems(null);
    }, [allowDesignMode, appController, designMode, theme]);

    return (
      <>
        <WindowContainer
          layout={layout}
          onLayoutChange={(newLayout) => {
            onLayoutChange &&
              onLayoutChange(
                newLayout.map((l) => ({ layout: l, microApp: windowApps.current[l.i] })),
              );
            setLayout(newLayout);
          }}
          designMode={designMode}
        >
          {layout.map((l) => {
            const microApp: MicroAppManifest | undefined = windowApps.current[l.i];
            if (!microApp) {
              return null;
            }
            return (
              <microApp.Component
                key={l.i}
                onClose={() => {
                  const newLayout = layout.filter((l2) => l2.i !== l.i);
                  console.log(layout, newLayout);
                  onLayoutChange &&
                    onLayoutChange(
                      newLayout.map((l) => ({ layout: l, microApp: windowApps.current[l.i] })),
                    );
                  setLayout(newLayout);
                  delete windowApps.current[l.i];
                }}
              />
            );
          })}
        </WindowContainer>
        {layout.length === 0 && allowDesignMode && (
          <Box
            sx={{
              position: 'absolute',
              top: '50%',
              left: '50%',
              transform: 'translate(-50%, -50%)',
              justifyContent: 'center',
              alignItems: 'center',
            }}
          >
            <Typography variant="h6">
              Click <DesignModeIcon /> in the app bar to start customizing the layout
            </Typography>
          </Box>
        )}
        {designMode && (
          <>
            <Fab
              color="primary"
              sx={{ position: 'fixed', right: '2vw', bottom: '2vw' }}
              onClick={(e) => setAddMenuAnchor(e.currentTarget)}
            >
              <AddIcon />
            </Fab>
            <Menu
              anchorEl={addMenuAnchor}
              anchorOrigin={{ vertical: 'center', horizontal: 'center' }}
              transformOrigin={{ vertical: 'bottom', horizontal: 'right' }}
              open={!!addMenuAnchor}
              onClose={() => setAddMenuAnchor(null)}
            >
              {appRegistry.map((manifest) => (
                <MenuItem
                  key={manifest.appId}
                  onClick={() => {
                    const newKey = `window-${windowId.current}`;
                    windowApps.current[newKey] = manifest;
                    ++windowId.current;
                    const newLayout = [...layout, { i: newKey, x: 0, y: 0, w: 2, h: 2 }];
                    onLayoutChange &&
                      onLayoutChange(
                        newLayout.map((l) => ({ layout: l, microApp: windowApps.current[l.i] })),
                      );
                    React.startTransition(() => setLayout(newLayout));
                    setAddMenuAnchor(null);
                  }}
                >
                  {manifest.displayName}
                </MenuItem>
              ))}
            </Menu>
          </>
        )}
      </>
    );
  },
);

interface SavedWorkspaceLayout {
  layout: WindowLayout;
  appId: string;
}

/**
 * A workspace that saves the state into `localStorage`.
 */
export interface LocallyPersistentWorkspaceProps
  extends Omit<WorkspaceProps, 'initialWindows' | 'onLayoutChange'> {
  /**
   * Default list of windows when there is nothing saved yet.
   */
  defaultWindows: InitialWindow[];
  storageKey: string;
}

export const LocallyPersistentWorkspace = ({
  defaultWindows,
  storageKey,
  appRegistry = [],
  ...otherProps
}: LocallyPersistentWorkspaceProps) => {
  const initialWindows = React.useMemo<InitialWindow[]>(() => {
    const json = localStorage.getItem(storageKey);
    if (!json) {
      return defaultWindows;
    }
    const saved: SavedWorkspaceLayout[] = JSON.parse(json);
    const loadedLayout: InitialWindow[] = [];
    for (const s of saved) {
      const microApp = appRegistry.find((app) => app.appId === s.appId);
      if (!microApp) {
        console.warn(`fail to load micro app [${s.appId}]`);
        continue;
      }
      loadedLayout.push({ layout: s.layout, microApp });
    }
    return loadedLayout;
  }, [defaultWindows, storageKey, appRegistry]);

  return (
    <Workspace
      initialWindows={initialWindows}
      appRegistry={appRegistry}
      onLayoutChange={(newLayout) => {
        localStorage.setItem(
          storageKey,
          JSON.stringify(
            newLayout.map<SavedWorkspaceLayout>((l) => ({
              layout: l.layout,
              appId: l.microApp.appId,
            })),
          ),
        );
      }}
      {...otherProps}
    />
  );
};
