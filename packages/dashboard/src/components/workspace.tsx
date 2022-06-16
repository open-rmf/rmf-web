import AddIcon from '@mui/icons-material/Add';
import { Fab, Menu, MenuItem } from '@mui/material';
import React from 'react';
import { WindowContainer, WindowLayout } from 'react-components';
import { AppRegistry, MicroAppProps } from './micro-app';

export interface WorkspaceWindow {
  key: string;
  app: React.ComponentType<MicroAppProps>;
}

export interface WorkspaceState {
  layout: WindowLayout[];
  windows: WorkspaceWindow[];
}

export interface WorkspaceProps {
  designMode: boolean;
  state: WorkspaceState;
  onStateChange?: (state: WorkspaceState) => void;
}

export function Workspace({ designMode, state, onStateChange }: WorkspaceProps): JSX.Element {
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
        {state.windows.map((w) => (
          <w.app
            key={w.key}
            onClose={() => {
              onStateChange &&
                onStateChange({
                  layout: state.layout.filter((l) => l.i !== w.key),
                  windows: state.windows.filter((w2) => w2.key !== w.key),
                });
            }}
          />
        ))}
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
        {Object.entries(AppRegistry).map(([name, app]) => (
          <MenuItem
            key={name}
            onClick={() => {
              const newKey = `${name}-${state.layout.length}`;
              onStateChange &&
                onStateChange({
                  layout: [...state.layout, { i: newKey, x: 0, y: 0, w: 2, h: 2 }],
                  windows: [...state.windows, { key: newKey, app }],
                });
              setAddMenuAnchor(null);
            }}
          >
            {name}
          </MenuItem>
        ))}
      </Menu>
    </>
  );
}

export const WorkspaceManager: Record<string, WorkspaceState> = {};
