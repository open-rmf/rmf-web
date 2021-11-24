import { Meta, Story } from '@storybook/react';
import type { FleetState, Lift } from 'api-client';
import React from 'react';
import { LayersControl } from 'react-leaflet';
import ColorManager from '../color-manager';
import { makeLift } from '../lifts/test-utils.spec';
import { Place } from '../place';
import { makeRobot } from '../robots/test-utils.spec';
import { useAsync } from '../use-async';
import { AffineImageOverlay } from './affine-image-overlay';
import DoorsOverlay from './doors-overlay';
import { LiftsOverlay } from './lifts-overlay';
import { LMap, LMapProps } from './map';
import { RobotData, RobotsOverlay } from './robots-overlay';
import { makeTrajectoryData, officeMap } from './test-utils.spec';
import { TrajectoriesOverlay } from './trajectories-overlay';
import { affineImageBounds, loadAffineImage } from './utils';
import { WaypointsOverlay } from './waypoints-overlay';
import { WorkcellData, WorkcellsOverlay } from './workcells-overlay';

export default {
  title: 'Map/Schedule Visualizer',
  component: LMap,
  parameters: { layout: 'fullscreen' },
} as Meta;

const colorManager = new ColorManager();
const dispensers: WorkcellData[] = [{ guid: 'test_dispenser', location: [18, -9] }];
const ingestors: WorkcellData[] = [{ guid: 'test_ingestor', location: [16, -9] }];
const fleetState: FleetState = {
  name: 'test_fleet',
  robots: [
    makeRobot({
      name: 'test_robot',
      location: { x: 20, y: -10, yaw: 0, level_name: 'L1', index: 0, t: { sec: 0, nanosec: 0 } },
    }),
  ],
};
const trajectories = [makeTrajectoryData()];
const waypoints: Place[] = [
  {
    level: 'L1',
    vertex: {
      x: 19.89569854736328,
      y: -3.4071500301361084,
      name: 'lounge',
      params: [],
    },
  },
];
const lifts: Lift[] = [
  makeLift({
    name: 'test_lift',
    ref_x: 2,
    ref_y: -4,
    ref_yaw: Math.PI / 4,
    width: 2,
    depth: 1,
    levels: ['L1', 'L2'],
    doors: [],
  }),
];

export const ScheduleVisualizer: Story<LMapProps> = () => {
  const safeAsync = useAsync();
  const levels = React.useMemo(
    () => [...officeMap.levels].sort((a, b) => a.name.localeCompare(b.name)),
    [],
  );
  const [currentLevel, setCurrentLevel] = React.useState(levels[0]);
  const [images, setImages] = React.useState<Record<string, HTMLImageElement>>({});
  const [levelBounds, setLevelBounds] = React.useState<Record<string, L.LatLngBoundsExpression>>(
    {},
  );
  const bounds = React.useMemo(() => levelBounds[currentLevel.name], [levelBounds, currentLevel]);
  const [robots, setRobots] = React.useState<RobotData[]>([]);

  React.useEffect(() => {
    (async () => {
      const promises = fleetState.robots.map(async (r) => ({
        fleet: fleetState.name,
        name: r.name,
        model: r.model,
        footprint: 0.5,
        color: await colorManager.robotPrimaryColor(fleetState.name, r.name, r.model),
      }));
      setRobots(await safeAsync(Promise.all(promises)));
    })();
  }, [safeAsync]);

  React.useEffect(() => {
    (async () => {
      const images = await safeAsync(Promise.all(levels.map((l) => loadAffineImage(l.images[0]))));
      setImages(
        levels.reduce<Record<string, HTMLImageElement>>((acc, l, idx) => {
          acc[l.name] = images[idx];
          return acc;
        }, {}),
      );
    })();
  }, [levels, safeAsync]);

  React.useEffect(() => {
    const bounds = levels.reduce<Record<string, L.LatLngBoundsExpression>>((acc, l) => {
      const imageEl = images[l.name];
      if (!imageEl) return acc;
      acc[l.name] = affineImageBounds(l.images[0], imageEl.naturalWidth, imageEl.naturalHeight);
      return acc;
    }, {});
    setLevelBounds(bounds);
  }, [images, levels]);

  const baseLayerHandler = (levelName: string): L.LeafletEventHandlerFnMap | undefined => {
    return {
      add: () => setCurrentLevel(levels.find((l) => l.name === levelName) || levels[0]),
      remove: () => setCurrentLevel(levels.find((l) => l.name === levelName) || levels[0]),
    };
  };

  return bounds ? (
    <div style={{ width: '100vw', height: '100vh', padding: 0, margin: 0 }}>
      <LMap bounds={bounds} zoomDelta={0.5} zoomSnap={0.5}>
        <LayersControl position="topleft">
          {officeMap.levels.map((level) => (
            <LayersControl.BaseLayer
              key={level.name}
              name={level.name}
              checked={currentLevel === level}
            >
              <AffineImageOverlay
                bounds={levelBounds[level.name]}
                image={level.images[0]}
                eventHandlers={baseLayerHandler(level.name)}
              />
            </LayersControl.BaseLayer>
          ))}
          <LayersControl.Overlay name="Waypoints" checked>
            <WaypointsOverlay bounds={bounds} waypoints={waypoints} />
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Dispensers" checked>
            <WorkcellsOverlay bounds={bounds} workcells={dispensers} />
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Ingestors" checked>
            <WorkcellsOverlay bounds={bounds} workcells={ingestors} />
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Lifts" checked>
            <LiftsOverlay bounds={bounds} lifts={lifts} currentLevel={currentLevel.name} />
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Doors" checked>
            <DoorsOverlay bounds={bounds} doors={currentLevel.doors} />
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Trajectories" checked>
            <TrajectoriesOverlay bounds={bounds} trajectoriesData={trajectories} />
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Robots" checked>
            <RobotsOverlay
              bounds={bounds}
              robots={robots}
              getRobotState={(_fleet, robot) => {
                return fleetState.robots.find((r) => r.name === robot) || null;
              }}
            />
          </LayersControl.Overlay>
        </LayersControl>
      </LMap>
    </div>
  ) : (
    <></>
  );
};
