import AddIcon from '@mui/icons-material/Add';
import DesignModeIcon from '@mui/icons-material/AutoFixNormal';
import { Box, Fab, IconButton, Menu, MenuItem, Typography, useTheme } from '@mui/material';
import React from 'react';
import { WindowContainer, WindowLayout } from 'react-components';
import { AppControllerContext } from './app-contexts';
import { AppRegistry } from './app-registry';

export interface WorkspaceWindow {
  key: string;
  appName: keyof typeof AppRegistry;
}

export interface WorkspaceState {
  layout: WindowLayout[];
  windows: WorkspaceWindow[];
}

export interface WorkspaceProps {
  state: WorkspaceState;
  designMode?: boolean;
  onStateChange?: (state: WorkspaceState) => void;
}

export function Workspace({
  state,
  designMode = false,
  onStateChange,
}: WorkspaceProps): JSX.Element {
  const [addMenuAnchor, setAddMenuAnchor] = React.useState<HTMLElement | null>(null);

  return (
    <>
      <WindowContainer
        layout={state.layout}
        onLayoutChange={(newLayout) =>
          onStateChange && onStateChange({ ...state, layout: newLayout })
        }
        designMode={designMode}
      >
        {state.windows.map((w) => {
          const MicroApp = AppRegistry[w.appName] || null;
          return MicroApp ? (
            <MicroApp
              key={w.key}
              onClose={() => {
                onStateChange &&
                  onStateChange({
                    layout: state.layout.filter((l) => l.i !== w.key),
                    windows: state.windows.filter((w2) => w2.key !== w.key),
                  });
              }}
            />
          ) : (
            <div></div>
          );
        })}
      </WindowContainer>
      {designMode && (
        <Fab
          color="primary"
          sx={{ position: 'absolute', right: '2vw', bottom: '2vw' }}
          onClick={(e) => setAddMenuAnchor(e.currentTarget)}
        >
          <AddIcon />
        </Fab>
      )}
      <Menu
        anchorEl={addMenuAnchor}
        anchorOrigin={{ vertical: 'center', horizontal: 'center' }}
        transformOrigin={{ vertical: 'bottom', horizontal: 'right' }}
        open={!!addMenuAnchor}
        onClose={() => setAddMenuAnchor(null)}
      >
        {Object.keys(AppRegistry).map((appName) => (
          <MenuItem
            key={appName}
            onClick={() => {
              const newKey = `${appName}-${state.layout.length}`;
              onStateChange &&
                onStateChange({
                  layout: [...state.layout, { i: newKey, x: 0, y: 0, w: 2, h: 2 }],
                  windows: [
                    ...state.windows,
                    { key: newKey, appName: appName as keyof typeof AppRegistry },
                  ],
                });
              setAddMenuAnchor(null);
            }}
          >
            {appName}
          </MenuItem>
        ))}
      </Menu>
    </>
  );
}

export interface ManagedWorkspaceProps {
  workspaceId: string;
}

export function ManagedWorkspace({ workspaceId }: ManagedWorkspaceProps) {
  const theme = useTheme();
  const appController = React.useContext(AppControllerContext);
  const [workspaceState, setWorkspaceState] = React.useState<WorkspaceState>(() => {
    const json = localStorage.getItem(`workspace-${workspaceId}`);
    return json ? JSON.parse(json) : { layout: [], windows: [] };
  });
  const [designMode, setDesignMode] = React.useState(false);

  React.useEffect(() => {
    appController.setExtraAppbarIcons(
      <IconButton
        color="inherit"
        sx={{ opacity: designMode ? undefined : theme.palette.action.disabledOpacity }}
        onClick={() => setDesignMode((prev) => !prev)}
      >
        <DesignModeIcon />
      </IconButton>,
    );
    return () => appController.setExtraAppbarIcons(null);
  }, [appController, designMode, theme]);

  return (
    <Box component="div" sx={{ display: 'contents', position: 'relative' }}>
      <Workspace
        state={workspaceState}
        onStateChange={(newState) => {
          setWorkspaceState(newState);
          localStorage.setItem(`workspace-${workspaceId}`, JSON.stringify(newState));
        }}
        designMode={designMode}
      />
      {workspaceState.windows.length === 0 && (
        <Box
          component="div"
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
    </Box>
  );
}
