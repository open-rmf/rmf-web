/* istanbul ignore file */

import { Dispenser, Ingestor } from 'api-client';
import Debug from 'debug';
import * as L from 'leaflet';
import React from 'react';
import {
  affineImageBounds,
  AffineImageOverlay,
  ColorManager,
  DoorsOverlay,
  LiftsOverlay,
  LMap,
  loadAffineImage,
  RobotData,
  RobotsOverlay,
  TrajectoriesOverlay,
  TrajectoryData,
  TrajectoryTimeControl,
  useAsync,
  WaypointsOverlay,
  WorkcellData,
  WorkcellsOverlay,
} from 'react-components';
import { makeStyles } from '@material-ui/core';
import { AttributionControl, LayersControl, Pane } from 'react-leaflet';
import * as RmfModels from 'rmf-models';
import appConfig from '../../app-config';
import { NegotiationTrajectoryResponse } from '../../managers/negotiation-status-manager';
import { ResourcesContext } from '../app-contexts';
import { PlacesContext, RmfIngressContext } from '../rmf-app';

const scheduleVisualizerStyle = makeStyles((theme) => ({
  map: {
    backgroundColor: theme.palette.background.default,
  },
}));

const debug = Debug('ScheduleVisualizer');
const TrajectoryUpdateInterval = 2000;
// schedule visualizer manages it's own settings so that it doesn't cause a re-render
// of the whole app when it changes.
const SettingsKey = 'scheduleVisualizerSettings';
const colorManager = new ColorManager();

export interface ScheduleVisualizerProps extends React.PropsWithChildren<{}> {
  buildingMap: RmfModels.BuildingMap;
  negotiationTrajStore?: Record<string, NegotiationTrajectoryResponse>;
  dispensers?: Dispenser[];
  ingestors?: Ingestor[];
  doorStates?: Record<string, RmfModels.DoorState>;
  liftStates?: Record<string, RmfModels.LiftState>;
  fleetStates?: Record<string, RmfModels.FleetState>;
  /**
   * default: 'normal'
   */
  mode?: 'normal' | 'negotiation';
  onDoorClick?: (ev: React.MouseEvent, door: string) => void;
  onLiftClick?: (ev: React.MouseEvent, lift: string) => void;
  onRobotClick?: (ev: React.MouseEvent, fleet: string, robot: string) => void;
  onDispenserClick?: (ev: React.MouseEvent, guid: string) => void;
  onIngestorClick?: (ev: React.MouseEvent, guid: string) => void;
}

interface ScheduleVisualizerSettings {
  trajectoryTime: number;
}

export default function ScheduleVisualizer({
  buildingMap,
  negotiationTrajStore = {},
  dispensers = [],
  ingestors = [],
  doorStates = {},
  liftStates = {},
  fleetStates = {},
  mode = 'normal',
  onDoorClick,
  onLiftClick,
  onRobotClick,
  onDispenserClick,
  onIngestorClick,
  children,
}: ScheduleVisualizerProps): JSX.Element | null {
  debug('render');
  const safeAsync = useAsync();
  const classes = scheduleVisualizerStyle();
  const levels = React.useMemo(
    () => [...buildingMap.levels].sort((a, b) => a.name.localeCompare(b.name)),
    [buildingMap],
  );
  const [currentLevel, setCurrentLevel] = React.useState(levels[0]);
  const [images, setImages] = React.useState<Record<string, HTMLImageElement>>({});
  const [levelBounds, setLevelBounds] = React.useState<Record<string, L.LatLngBoundsExpression>>(
    {},
  );
  const bounds = React.useMemo(() => levelBounds[currentLevel.name], [levelBounds, currentLevel]);

  const [robots, setRobots] = React.useState<RobotData[]>([]);
  const { current: robotsStore } = React.useRef<Record<string, RobotData>>({});

  // FIXME: trajectory manager should handle the tokens
  const authenticator = appConfig.authenticator;

  const [trajectories, setTrajectories] = React.useState<TrajectoryData[]>([]);
  const { trajectoryManager: trajManager } = React.useContext(RmfIngressContext) || {};

  const [
    scheduleVisualizerSettings,
    setScheduleVisualizerSettings,
  ] = React.useState<ScheduleVisualizerSettings>(() => {
    const settings = window.localStorage.getItem(SettingsKey);
    return settings ? JSON.parse(settings) : { trajectoryTime: 60000 /* 1 min */ };
  });
  const trajectoryTime = scheduleVisualizerSettings.trajectoryTime;
  const trajectoryAnimScale = trajectoryTime / (0.9 * TrajectoryUpdateInterval);

  const negoTrajectories = React.useMemo<TrajectoryData[]>(() => {
    if (mode !== 'negotiation') return [];
    const negoTrajs = negotiationTrajStore[currentLevel.name];
    return negoTrajs
      ? negoTrajs.values.map((v) => ({
          trajectory: v,
          color: 'orange',
          animationScale: trajectoryAnimScale,
          loopAnimation: false,
          conflict: false,
        }))
      : [];
  }, [mode, negotiationTrajStore, currentLevel, trajectoryAnimScale]);

  const renderedTrajectories = React.useMemo(() => {
    switch (mode) {
      case 'normal':
        return trajectories;
      case 'negotiation':
        return negoTrajectories;
    }
  }, [mode, trajectories, negoTrajectories]);

  React.useEffect(() => {
    let interval: number;
    let cancel = false;

    if (mode !== 'normal') return;

    const updateTrajectory = async () => {
      debug('updating trajectories');

      if (cancel || !trajManager) return;

      const resp = await trajManager.latestTrajectory({
        request: 'trajectory',
        param: {
          map_name: currentLevel.name,
          duration: trajectoryTime,
          trim: true,
        },
        token: authenticator.token,
      });
      const flatConflicts = resp.conflicts.flatMap((c) => c);

      debug('set trajectories');
      setTrajectories(
        resp.values.map((v) => ({
          trajectory: v,
          color: 'green',
          conflict: flatConflicts.includes(v.id),
          animationScale: trajectoryAnimScale,
          loopAnimation: false,
        })),
      );
    };

    updateTrajectory();
    interval = window.setInterval(updateTrajectory, TrajectoryUpdateInterval);
    debug(`created trajectory update interval ${interval}`);

    return () => {
      cancel = true;
      clearInterval(interval);
      debug(`cleared interval ${interval}`);
    };
  }, [trajManager, currentLevel, trajectoryTime, mode, authenticator.token, trajectoryAnimScale]);

  const resourceManager = React.useContext(ResourcesContext);

  const [dispensersData, setDispensersData] = React.useState<WorkcellData[]>([]);
  React.useEffect(() => {
    const dispenserManager = resourceManager?.dispensers;
    if (!dispenserManager) return;
    (async () => {
      const dispenserResources = dispenserManager.dispensers;
      const availableData = dispensers.filter((wc) => wc.guid in dispenserResources);
      const promises = availableData.map((wc) => dispenserManager.getIconPath(wc.guid));
      const icons = await safeAsync(Promise.all(promises));
      setDispensersData(
        availableData.map((wc, i) => ({
          guid: wc.guid,
          location: [
            dispenserResources[wc.guid].location.x,
            dispenserResources[wc.guid].location.y,
          ],
          iconPath: icons[i] || undefined,
        })),
      );
    })();
  }, [safeAsync, resourceManager?.dispensers, dispensers]);

  const [ingestorsData, setIngestorsData] = React.useState<WorkcellData[]>([]);
  React.useEffect(() => {
    const dispenserManager = resourceManager?.dispensers;
    if (!dispenserManager) return;
    (async () => {
      const dispenserResources = dispenserManager.dispensers;
      const availableData = ingestors.filter((wc) => wc.guid in dispenserResources);
      const promises = availableData.map((wc) => dispenserManager.getIconPath(wc.guid));
      const icons = await safeAsync(Promise.all(promises));
      setIngestorsData(
        availableData.map((wc, i) => ({
          guid: wc.guid,
          location: [
            dispenserResources[wc.guid].location.x,
            dispenserResources[wc.guid].location.y,
          ],
          iconPath: icons[i] || undefined,
        })),
      );
    })();
  }, [safeAsync, resourceManager?.dispensers, ingestors]);

  const places = React.useContext(PlacesContext);
  const waypoints = React.useMemo(() => places.filter((p) => p.vertex.name.length > 0), [places]);

  React.useEffect(() => {
    (async () => {
      const promises = Object.values(fleetStates).flatMap((fleetState) =>
        fleetState.robots.map(async (r) => {
          const robotId = `${fleetState.name}/${r.name}`;
          if (robotId in robotsStore) return;
          robotsStore[robotId] = {
            fleet: fleetState.name,
            name: r.name,
            model: r.model,
            footprint: 0.5,
            color: await colorManager.robotPrimaryColor(fleetState.name, r.name, r.model),
            iconPath:
              (await resourceManager?.robots.getIconPath(fleetState.name, r.model)) || undefined,
          };
        }),
      );
      await safeAsync(Promise.all(promises));
    })();
  }, [safeAsync, fleetStates, robotsStore, resourceManager]);

  React.useEffect(() => {
    const newRobots = Object.values(fleetStates).flatMap((fleetState) =>
      fleetState.robots
        .filter(
          (r) =>
            r.location.level_name === currentLevel.name &&
            `${fleetState.name}/${r.name}` in robotsStore,
        )
        .map((r) => robotsStore[`${fleetState.name}/${r.name}`]),
    );
    setRobots(newRobots);
  }, [safeAsync, fleetStates, robotsStore, currentLevel]);

  React.useEffect(() => {
    (async () => {
      const images = await safeAsync(Promise.all(levels.map((l) => loadAffineImage(l.images[0]))));
      setImages(
        levels.reduce((acc, l, idx) => {
          acc[l.name] = images[idx];
          return acc;
        }, {} as Record<string, HTMLImageElement>),
      );
    })();
  }, [levels, safeAsync]);

  React.useEffect(() => {
    const bounds = levels.reduce((acc, l) => {
      const imageEl = images[l.name];
      if (!imageEl) return acc;
      acc[l.name] = affineImageBounds(l.images[0], imageEl.naturalWidth, imageEl.naturalHeight);
      return acc;
    }, {} as Record<string, L.LatLngBoundsExpression>);
    setLevelBounds(bounds);
  }, [images, levels]);

  return bounds ? (
    <LMap
      id="schedule-visualizer"
      attributionControl={false}
      minZoom={0}
      maxZoom={8}
      zoomDelta={0.5}
      zoomSnap={0.5}
      bounds={bounds}
      className={classes.map}
    >
      <AttributionControl position="bottomright" prefix="OSRC-SG" />
      <LayersControl
        position="topleft"
        onbaselayerchange={(ev) =>
          setCurrentLevel(levels.find((l) => l.name === ev.name) || levels[0])
        }
      >
        {buildingMap.levels.map((level) => (
          <LayersControl.BaseLayer
            key={level.name}
            name={level.name}
            checked={currentLevel === level}
          >
            <AffineImageOverlay bounds={levelBounds[level.name]} image={level.images[0]} />
          </LayersControl.BaseLayer>
        ))}

        <LayersControl.Overlay name="Trajectories" checked>
          <Pane>
            <TrajectoriesOverlay bounds={bounds} trajectoriesData={renderedTrajectories} />
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
              getRobotState={(fleet, robot) => {
                const state = fleetStates[fleet].robots.find((r) => r.name === robot);
                return state || null;
              }}
              onRobotClick={onRobotClick}
            />
          </Pane>
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Dispensers" checked>
          <Pane>
            <WorkcellsOverlay
              bounds={bounds}
              workcells={dispensersData}
              onWorkcellClick={onDispenserClick}
            />
          </Pane>
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Ingestors" checked>
          <Pane>
            <WorkcellsOverlay
              bounds={bounds}
              workcells={ingestorsData}
              onWorkcellClick={onIngestorClick}
            />
          </Pane>
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Lifts" checked>
          <Pane>
            <LiftsOverlay
              bounds={bounds}
              currentLevel={currentLevel.name}
              lifts={buildingMap.lifts}
              liftStates={liftStates}
              onLiftClick={onLiftClick}
            />
          </Pane>
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Doors" checked>
          <Pane>
            <DoorsOverlay
              bounds={bounds}
              doors={currentLevel.doors}
              doorStates={doorStates}
              onDoorClick={onDoorClick}
            />
          </Pane>
        </LayersControl.Overlay>
      </LayersControl>

      <TrajectoryTimeControl
        position="topleft"
        value={trajectoryTime}
        min={60000}
        max={600000}
        onChange={(_ev, newValue) =>
          setScheduleVisualizerSettings((prev) => {
            const newSettings = { ...prev, trajectoryTime: newValue };
            window.localStorage.setItem(SettingsKey, JSON.stringify(newSettings));
            return newSettings;
          })
        }
      />
      {children}
    </LMap>
  ) : null;
}
