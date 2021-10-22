import React from 'react';
import { Window } from 'react-components';
import { allWindows, ManagedWindowProps, WindowClass } from '../window';
import ScheduleVisualizer from './schedule-visualizer';

export const ScheduleVisualizerWindow: React.FC<ManagedWindowProps> = React.forwardRef(
  ({ ...otherProps }, ref) => {
    return (
      <Window ref={ref} title="Map" {...otherProps}>
        <ScheduleVisualizer />
      </Window>
    );
  },
);

export default ScheduleVisualizerWindow;

export const scheduleVisualizerWindowClass = new WindowClass(
  'Interactive Map',
  ScheduleVisualizerWindow,
  {
    x: 0,
    y: 0,
    w: 4,
    h: 4,
    minW: 4,
    minH: 4,
  },
);

allWindows['scheduleVisualizer'] = scheduleVisualizerWindowClass;
