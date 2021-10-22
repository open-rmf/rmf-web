/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import React from 'react';
import { WindowManager, WindowManagerProps } from 'react-components';
import { DoorsWindow, doorsWindowClass } from '../doors';
import { LiftsWindow, liftsWindowClass } from '../lifts';
import { ScheduleVisualizerWindow, scheduleVisualizerWindowClass } from '../schedule-visualizer';
import { WorkcellsWindow, workcellsWindowClass } from '../workcells';

const useStyles = makeStyles({
  windowManager: {
    width: '100%',
    height: '100%',
  },
});

export const InfrastructurePage: React.FC<{}> = () => {
  const classes = useStyles();
  const layouts = React.useMemo<WindowManagerProps['layouts']>(
    () => ({
      xl: [
        scheduleVisualizerWindowClass.createLayout('xl', {
          i: 'infrastructure-scheduleVisualizer',
          x: 0,
          y: 0,
          w: 8,
          h: 12,
        }),
        doorsWindowClass.createLayout('xl', { i: 'infrastructure-doors', x: 8 }),
        liftsWindowClass.createLayout('xl', { i: 'infrastructure-lifts', x: 8, y: 4 }),
        workcellsWindowClass.createLayout('xl', { i: 'infrastructure-workcells', x: 8, y: 8 }),
      ],
    }),
    [],
  );
  return (
    <WindowManager layouts={layouts} className={classes.windowManager}>
      <ScheduleVisualizerWindow key="infrastructure-scheduleVisualizer" />
      <DoorsWindow key="infrastructure-doors" />
      <LiftsWindow key="infrastructure-lifts" />
      <WorkcellsWindow key="infrastructure-workcells" />
    </WindowManager>
  );
};

export default InfrastructurePage;
