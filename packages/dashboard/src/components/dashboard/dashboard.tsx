/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import React from 'react';
import { WindowManager, WindowManagerProps } from 'react-components';
import { DoorsWindow, doorsWindowClass } from '../doors';
import { LiftsWindow, liftsWindowClass } from '../lifts';
import { WorkcellsWindow, workcellsWindowClass } from '../workcells';

const useStyles = makeStyles({
  windowManager: {
    width: '100%',
    height: '100%',
  },
});

export const Dashboard: React.FC<{}> = () => {
  const classes = useStyles();
  const layouts = React.useMemo<WindowManagerProps['layouts']>(
    () => ({
      xl: [
        doorsWindowClass.createLayout('xl', { i: 'doors', x: 8 }),
        liftsWindowClass.createLayout('xl', { i: 'lifts', x: 8, y: 4 }),
        workcellsWindowClass.createLayout('xl', { i: 'workcells', x: 8, y: 8 }),
      ],
    }),
    [],
  );
  return (
    <WindowManager layouts={layouts} className={classes.windowManager}>
      <DoorsWindow key="doors" />
      <LiftsWindow key="lifts" />
      <WorkcellsWindow key="workcells" />
    </WindowManager>
  );
};

export default Dashboard;
