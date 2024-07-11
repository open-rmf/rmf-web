import { Typography, useTheme } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { makeLayout } from './test-utils.spec';
import { Window } from './window';
import { WindowContainer, WindowContainerProps } from './window-container';

export default {
  title: 'Window/Demo',
} satisfies Meta;

const defaultLayout = makeLayout();

export const Demo: StoryFn<WindowContainerProps> = () => {
  const [layout, setLayout] = React.useState(defaultLayout);
  const [designMode, setDesignMode] = React.useState(true);
  const counter = React.useRef(defaultLayout.length - 1);
  const theme = useTheme();
  return (
    <div>
      <button
        onClick={() => {
          setLayout(defaultLayout);
          counter.current = defaultLayout.length - 1;
        }}
      >
        Reset
      </button>
      <button
        onClick={() =>
          setLayout((prev) => [
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
      <WindowContainer
        designMode={designMode}
        layout={layout}
        onLayoutChange={(newLayout) => setLayout(newLayout)}
        style={{
          height: '90vh',
          background: theme.palette.background.default,
        }}
      >
        {layout.map(({ i }) => (
          <Window
            key={i}
            title={`Window ${i}`}
            onClose={() => setLayout((prev) => prev.filter((l) => l.i !== i))}
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
      </WindowContainer>
    </div>
  );
};
