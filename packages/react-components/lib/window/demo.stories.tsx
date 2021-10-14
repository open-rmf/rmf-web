import { useTheme } from '@material-ui/core';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { makeTask } from '../tasks/test-data.spec';
import { Window } from './window';
import { WindowContainer, WindowContainerProps } from './window-container';
import { useWindowManager, WindowPreference } from './window-manager';

export default {
  title: 'Window/Demo',
} as Meta;

const windowClasses = {
  empty: new WindowPreference('empty', 'Window', { xl: { w: 2, h: 4 } }),
  taskTable: new WindowPreference('taskTable', 'Tasks', { xl: { w: 8, h: 4, minW: 8, minH: 4 } }),
};

const tasks = [makeTask('active_task', 3, 3), makeTask('active_task_2', 4, 3)];
const defaultWindows = Array.from(new Array(4)).map(() => windowClasses.empty.createWindow());

export const Demo: Story<WindowContainerProps> = () => {
  const windowManager = useWindowManager(defaultWindows);
  const [designMode, setDesignMode] = React.useState(true);
  const theme = useTheme();
  return (
    <div>
      <div>
        {/* <button onClick={() => setLayouts(defaultLayouts)}>Reset</button> */}
        <button onClick={() => setDesignMode(!designMode)}>
          {designMode ? 'Normal Mode' : 'Design Mode'}
        </button>
      </div>
      <div>
        <button onClick={() => windowManager.addWindow(windowClasses.empty.createWindow())}>
          Add Empty
        </button>
      </div>
      <WindowContainer
        designMode={designMode}
        layouts={windowManager.layouts}
        onLayoutChange={windowManager.onLayoutChange}
        style={{ background: theme.palette.background.default, height: '95vh' }}
      >
        {Object.values(windowManager.windows).map(({ key, windowProps }) => (
          <Window
            {...windowProps}
            key={key}
            onClose={() => windowManager.removeWindow(key)}
          ></Window>
        ))}
      </WindowContainer>
    </div>
  );
};
