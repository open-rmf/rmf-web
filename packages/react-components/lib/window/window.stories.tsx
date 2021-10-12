import { Typography, useTheme } from '@material-ui/core';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { makeLayouts } from './test-utils.spec';
import { Window, WindowProps } from './window';
import { WindowContainer, WindowContainerProps } from './window-container';
import { WindowToolbar } from './window-toolbar';

export default {
  title: 'Window/Window',
  component: Window,
} as Meta;

const defaultLayouts: WindowContainerProps['layouts'] = makeLayouts();

export const Default: Story<WindowProps> = () => {
  const [layouts, setLayouts] = React.useState(defaultLayouts);
  const theme = useTheme();
  return (
    <div>
      <button onClick={() => setLayouts(defaultLayouts)}>Reset</button>
      <WindowContainer
        layouts={layouts}
        onLayoutChange={(newLayout) => setLayouts(newLayout)}
        style={{ background: theme.palette.background.default, height: '95vh' }}
        compactType={null}
      >
        {defaultLayouts.map(({ i }) => (
          <Window key={i} header={<WindowToolbar title={`Window ${i}`} />}>
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

Default.storyName = 'Window';
