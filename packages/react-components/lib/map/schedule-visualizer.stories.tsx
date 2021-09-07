import { Meta, Story } from '@storybook/react';
import React from 'react';
import { LayersControl, Pane } from 'react-leaflet';
import * as RmfModels from 'rmf-models';
import ColorManager from '../color-manager';
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
const fleetState: RmfModels.FleetState = {
  name: 'test_fleet',
  robots: [
    makeRobot({
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
        levels.reduce((acc, l, idx) => {
          acc[l.name] = images[idx];
          return acc;
        }, {}),
      );
    })();
  }, [levels, safeAsync]);

  React.useEffect(() => {
    const bounds = levels.reduce((acc, l) => {
      const imageEl = images[l.name];
      if (!imageEl) return acc;
      acc[l.name] = affineImageBounds(l.images[0], imageEl.naturalWidth, imageEl.naturalHeight);
      return acc;
    }, {});
    setLevelBounds(bounds);
  }, [images, levels]);

  return bounds ? (
    <div style={{ width: '100vw', height: '100vh', padding: 0, margin: 0 }}>
      <LMap bounds={bounds} zoomDelta={0.5} zoomSnap={0.5}>
        <LayersControl
          position="topleft"
          onbaselayerchange={(ev) => setCurrentLevel(levels.find((l) => l.name === ev.name))}
        >
          {officeMap.levels.map((level) => (
            <LayersControl.BaseLayer
              key={level.name}
              name={level.name}
              checked={currentLevel === level}
            >
              <AffineImageOverlay bounds={levelBounds[level.name]} image={level.images[0]} />
            </LayersControl.BaseLayer>
          ))}
          {/* zIndex are ordered in reverse,
          i.e. in this case, trajectories will be on to top most pane and doors is on the bottom most pane*/}
          <LayersControl.Overlay name="Trajectories" checked>
            <Pane>
              <TrajectoriesOverlay bounds={bounds} trajectoriesData={trajectories} />
            </Pane>
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Waypoints" checked>
            <Pane>
              <WaypointsOverlay bounds={bounds} waypoints={waypoints} />
            </Pane>
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Robots" checked>
            <Pane>
              <RobotsOverlay
                bounds={bounds}
                robots={robots}
                getRobotState={(fleet, robot) => fleetState.robots.find((r) => r.name === robot)}
              />
            </Pane>
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Dispensers" checked>
            <Pane>
              <WorkcellsOverlay bounds={bounds} workcells={dispensers} />
            </Pane>
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Ingestors" checked>
            <Pane>
              <WorkcellsOverlay bounds={bounds} workcells={ingestors} />
            </Pane>
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Lifts" checked>
            <Pane>
              <LiftsOverlay
                bounds={bounds}
                lifts={officeMap.lifts}
                currentLevel={currentLevel.name}
              />
            </Pane>
          </LayersControl.Overlay>
          <LayersControl.Overlay name="Doors" checked>
            <Pane>
              <DoorsOverlay bounds={bounds} doors={currentLevel.doors} />
            </Pane>
          </LayersControl.Overlay>
        </LayersControl>
      </LMap>
    </div>
  ) : null;
};
