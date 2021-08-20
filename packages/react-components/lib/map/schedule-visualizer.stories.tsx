import { Meta, Story } from '@storybook/react';
import React from 'react';
import { LayersControl, Pane } from 'react-leaflet';
import ColorManager from '../color-manager';
import { useAsync } from '../use-async';
import { AffineImageOverlay } from './affine-image-overlay';
import DoorsOverlay from './doors-overlay';
import { LiftsOverlay } from './lifts-overlay';
import { LMap, LMapProps } from './map';
import { RobotData, RobotsOverlay } from './robots-overlay';
import { dispensers, fleetState, ingestors, officeMap } from './test-utils.spec';
import { affineImageBounds, loadImage } from './utils';
import { WorkcellsOverlay } from './workcells-overlay';

export default {
  title: 'Schedule Visualizer',
  component: LMap,
  parameters: { layout: 'fullscreen' },
} as Meta;

const colorManager = new ColorManager();

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
        state: r,
        color: await colorManager.robotPrimaryColor(fleetState.name, r.name, r.model),
      }));
      setRobots(await safeAsync(Promise.all(promises)));
    })();
  }, [safeAsync]);

  React.useEffect(() => {
    (async () => {
      const images = await safeAsync(Promise.all(levels.map((l) => loadImage(l.images[0]))));
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
          <LayersControl.Overlay name="Doors" checked>
            <Pane>
              <DoorsOverlay bounds={bounds} doors={currentLevel.doors} />
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
          <LayersControl.Overlay name="Robots" checked>
            <Pane>
              <RobotsOverlay bounds={bounds} robots={robots} />
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
        </LayersControl>
      </LMap>
    </div>
  ) : (
    <></>
  );
};
