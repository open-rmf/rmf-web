import { Meta, Story } from '@storybook/react';
import 'leaflet/dist/leaflet.css';
import React from 'react';
import { Map as LMap } from 'react-leaflet';
import {
  TrajectoryTimeControl,
  TrajectoryTimeControlProps,
} from '../../components/schedule-visualizer/trajectory-time-control';

export default {
  title: 'ScheduleVisualizer/Trajectory Time Control',
  component: TrajectoryTimeControl,
} as Meta;

export const Default: Story<TrajectoryTimeControlProps> = () => {
  const [time, setTime] = React.useState(60000);
  return (
    <LMap
      style={{ width: 800, height: 450 }}
      bounds={[
        [0, 0],
        [1, 1],
      ]}
    >
      <TrajectoryTimeControl
        value={time}
        min={60000}
        max={600000}
        onChange={(_ev, newValue) => setTime(newValue)}
      />
    </LMap>
  );
};

Default.storyName = 'Trajectory Time Control';
