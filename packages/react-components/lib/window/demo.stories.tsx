import { Typography, useTheme } from '@material-ui/core';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { makeLayouts } from './test-utils.spec';
import { Window } from './window';
import { WindowManager, WindowManagerProps } from './window-manager';

export default {
  title: 'Window/Demo',
} as Meta;

const defaultLayouts: WindowManagerProps['layouts'] = makeLayouts();

export const Demo: Story<WindowManagerProps> = () => {
  const [layouts, setLayouts] = React.useState(defaultLayouts);
  const [designMode, setDesignMode] = React.useState(true);
  const counter = React.useRef(defaultLayouts.length - 1);
  const theme = useTheme();
  return (
    <div>
      <button
        onClick={() => {
          setLayouts(defaultLayouts);
          counter.current = defaultLayouts.length - 1;
        }}
      >
        Reset
      </button>
      <button
        onClick={() =>
          setLayouts((prev) => [
            ...prev,
            { i: (++counter.current).toString(), x: 0, y: 0, w: 2, h: 4, minW: 2, minH: 4 },
          ])
        }
      >
        Add
      </button>
      <button onClick={() => setDesignMode(!designMode)}>
        {designMode ? 'Normal Mode' : 'Design Mode'}
      </button>
      <WindowManager
        designMode={designMode}
        layouts={layouts}
        onLayoutChange={(newLayout) => setLayouts(newLayout)}
        style={{ background: theme.palette.background.default, height: '95vh' }}
      >
        {layouts.map(({ i }) => (
          <Window
            key={i}
            title={`Window ${i}`}
            onClose={() => setLayouts((prev) => prev.filter((l) => l.i !== i))}
          >
            <div
              style={{
                display: 'flex',
                height: '100%',
                justifyContent: 'center',
                alignItems: 'center',
              }}
            >
              <Typography style={{ textAlign: 'center' }}>{i}</Typography>
            </div>
          </Window>
        ))}
      </WindowManager>
    </div>
  );
};
