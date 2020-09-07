import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import React from 'react';
import { Map as LMap } from 'react-leaflet';
import ColorManager from '../../components/schedule-visualizer/colors';
import RobotsOverlay from '../../components/schedule-visualizer/robots-overlay';

function createRobots(count: number): RomiCore.RobotState[] {
  return [...Array(count)].map((_, i) => ({
    name: `robot${i}`,
    battery_percent: 100,
    location: {
      x: Math.random() * 20 - 10,
      y: Math.random() * 20 - 10,
      yaw: 0,
      t: { sec: 0, nanosec: 0 },
      level_name: 'L1',
    },
    mode: { mode: RomiCore.RobotMode.MODE_MOVING },
    model: '',
    path: [],
    task_id: '',
  }));
}

export default function RobotBenchmark(): React.ReactElement {
  const [numRobots, setNumRobots] = React.useState(100);
  const [robots, setRobots] = React.useState(() => createRobots(numRobots));
  const colorManager = React.useMemo(() => new ColorManager(), []);
  const fleets = React.useMemo<RomiCore.FleetState[]>(
    () => [
      {
        name: 'testFleet',
        robots: robots,
      },
    ],
    [robots],
  );

  return (
    <>
      <div style={{ display: 'flex', width: 200, alignItems: 'center' }}>
        <label htmlFor="numRobots">Number of Robots: </label>
        <input
          id="numRobots"
          type="number"
          style={{ width: '3em', marginLeft: 4 }}
          value={numRobots}
          onChange={e => {
            const num = parseInt(e.currentTarget.value);
            setNumRobots(num);
            num && setRobots(createRobots(num));
          }}
        />
      </div>
      <LMap
        style={{ width: 1000, height: 1000 }}
        attributionControl={false}
        crs={L.CRS.Simple}
        minZoom={4}
        maxZoom={8}
        zoomDelta={0.5}
        zoomSnap={0.5}
        bounds={new L.LatLngBounds([-10, -10], [10, 10])}
      >
        <RobotsOverlay
          bounds={new L.LatLngBounds([-10, -10], [10, 10])}
          colorManager={colorManager}
          conflictRobotNames={[]}
          currentFloorName="L1"
          fleets={fleets}
        />
      </LMap>
    </>
  );
}
