import React from 'react';
import { Window } from 'react-components';
import {
  allWindows,
  getWindowSettings,
  ManagedWindowProps,
  saveWindowSettings,
  WindowClass,
} from '../window';
import { ScheduleVisualizer, ScheduleVisualizerSettings } from './schedule-visualizer';

const defaultSettings: ScheduleVisualizerSettings = { trajectoryTime: 60000 /* 1 min */ };

export const ScheduleVisualizerWindow: React.FC<ManagedWindowProps> = React.forwardRef(
  ({ ...otherProps }, ref) => {
    const settings =
      getWindowSettings<ScheduleVisualizerSettings>(otherProps.key) || defaultSettings;
    return (
      <Window ref={ref} title="Map" {...otherProps}>
        <ScheduleVisualizer
          settings={settings}
          onSettingsChange={(newSettings) => saveWindowSettings(otherProps.key, newSettings)}
        />
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
