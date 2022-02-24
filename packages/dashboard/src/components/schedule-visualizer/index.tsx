/* istanbul ignore file */

import {
  BuildingMap,
  Dispenser,
  DoorState,
  FleetState,
  Ingestor,
  LiftState,
  RobotState,
} from 'api-client';
import Debug from 'debug';
import * as L from 'leaflet';
import React from 'react';
import {
  affineImageBounds,
  AffineImageOverlay,
  ColorManager,
  DoorsOverlay as DoorsOverlay_,
  LiftsOverlay as LiftsOverlay_,
  LMap,
  loadAffineImage,
  RobotData,
  RobotsOverlay as RobotsOverlay_,
  TrajectoriesOverlay as TrajectoriesOverlay_,
  TrajectoryData,
  TrajectoryTimeControl,
  useAsync,
  WaypointsOverlay as WaypointsOverlay_,
  WorkcellData,
  WorkcellsOverlay as WorkcellsOverlay_,
} from 'react-components';
import { AttributionControl, LayersControl } from 'react-leaflet';
import appConfig from '../../app-config';
import { NegotiationTrajectoryResponse } from '../../managers/negotiation-status-manager';
import { ResourcesContext } from '../app-contexts';
import { PlacesContext, RmfIngressContext } from '../rmf-app';

const DoorsOverlay = React.memo(DoorsOverlay_);
const LiftsOverlay = React.memo(LiftsOverlay_);
const RobotsOverlay = React.memo(RobotsOverlay_);
const TrajectoriesOverlay = React.memo(TrajectoriesOverlay_);
const WaypointsOverlay = React.memo(WaypointsOverlay_);
const WorkcellsOverlay = React.memo(WorkcellsOverlay_);

const debug = Debug('ScheduleVisualizer');
const TrajectoryUpdateInterval = 2000;
// schedule visualizer manages it's own settings so that it doesn't cause a re-render
// of the whole app when it changes.
const SettingsKey = 'scheduleVisualizerSettings';
const colorManager = new ColorManager();

export interface ScheduleVisualizerProps extends React.PropsWithChildren<{}> {
  buildingMap: BuildingMap;
  negotiationTrajStore?: Record<string, NegotiationTrajectoryResponse>;
  dispensers?: Dispenser[];
  ingestors?: Ingestor[];
  doorStates?: Record<string, DoorState>;
  liftStates?: Record<string, LiftState>;
  fleetStates?: Record<string, FleetState>;
  /**
   * default: 'normal'
   */
  mode?: 'normal' | 'negotiation';
  zoom?: number | undefined;
  onDoorClick?: (ev: React.MouseEvent, door: string) => void;
  onLiftClick?: (ev: React.MouseEvent, lift: string) => void;
  onRobotClick?: (ev: React.MouseEvent, fleet: string, robot: string) => void;
  onDispenserClick?: (ev: React.MouseEvent, guid: string) => void;
  onIngestorClick?: (ev: React.MouseEvent, guid: string) => void;
}

interface ScheduleVisualizerSettings {
  trajectoryTime: number;
}

export default React.forwardRef(function ScheduleVisualizer(
  {
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
  }: ScheduleVisualizerProps,
  ref,
): JSX.Element | null {
  debug('render');
  const safeAsync = useAsync();
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
  const [scheduleVisualizerSettings, setScheduleVisualizerSettings] =
    React.useState<ScheduleVisualizerSettings>(() => {
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
      const availableData = dispensers.filter(
        (wc) =>
          wc.guid in dispenserResources &&
          dispenserResources[wc.guid].location.level_name === currentLevel.name,
      );
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
  }, [safeAsync, resourceManager?.dispensers, dispensers, currentLevel.name]);

  const [ingestorsData, setIngestorsData] = React.useState<WorkcellData[]>([]);
  React.useEffect(() => {
    const dispenserManager = resourceManager?.dispensers;
    if (!dispenserManager) return;
    (async () => {
      const dispenserResources = dispenserManager.dispensers;
      const availableData = ingestors.filter(
        (wc) =>
          wc.guid in dispenserResources &&
          dispenserResources[wc.guid].location.level_name === currentLevel.name,
      );
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
  }, [safeAsync, resourceManager?.dispensers, ingestors, currentLevel]);

  const places = React.useContext(PlacesContext);
  const waypoints = React.useMemo(
    () => places.filter((p) => p.level === currentLevel.name && p.vertex.name.length > 0),
    [places, currentLevel],
  );

  React.useEffect(() => {
    (async () => {
      const promises = Object.values(fleetStates).flatMap((fleetState) => {
        const robotKey = fleetState.robots && Object.keys(fleetState.robots);
        const fleetName = fleetState.name ? fleetState.name : '';
        return robotKey?.map(async (r) => {
          const robotId = `${fleetState.name}/${r}`;
          if (robotId in robotsStore) return;
          robotsStore[robotId] = {
            fleet: fleetName,
            name: r,
            // no model name
            model: '',
            footprint: 0.5,
            color: await colorManager.robotPrimaryColor(fleetName, r, ''),
            iconPath: (await resourceManager?.robots.getIconPath(fleetName, r)) || undefined,
          };
        });
      });
      await safeAsync(Promise.all(promises));
      const newRobots = Object.values(fleetStates).flatMap((fleetState) => {
        const robotKey = fleetState.robots ? Object.keys(fleetState.robots) : [];
        return robotKey
          ?.filter(
            (r) =>
              fleetState.robots &&
              fleetState.robots[r].location?.map === currentLevel.name &&
              `${fleetState.name}/${r}` in robotsStore,
          )
          .map((r) => robotsStore[`${fleetState.name}/${r}`]);
      });
      setRobots(newRobots);
    })();
  }, [safeAsync, fleetStates, robotsStore, resourceManager, currentLevel]);

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

  const [layersUnChecked, setLayersUnChecked] = React.useState<Record<string, boolean>>({});
  const waypointsLayersRef: React.Ref<LayersControl.Overlay> = React.useRef(null);
  const registeredLayersHandlers = React.useRef(false);

  return bounds ? (
    <LMap
      ref={(cur) => {
        if (registeredLayersHandlers.current || !cur) return;
        cur.leafletElement.on('overlayadd', (ev: L.LayersControlEvent) =>
          setLayersUnChecked((prev) => ({ ...prev, [ev.name]: false })),
        );
        cur.leafletElement.on('overlayremove', (ev: L.LayersControlEvent) =>
          setLayersUnChecked((prev) => ({ ...prev, [ev.name]: true })),
        );
        registeredLayersHandlers.current = true;
        if (typeof ref === 'function') ref(cur);
        else if (ref) {
          ref.current = cur;
        }
      }}
      id="schedule-visualizer"
      attributionControl={false}
      minZoom={0}
      maxZoom={8}
      zoomDelta={0.5}
      zoomSnap={0.5}
      bounds={bounds}
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

        <LayersControl.Overlay
          ref={waypointsLayersRef}
          name="Waypoints"
          checked={!layersUnChecked['Waypoints']}
        >
          <WaypointsOverlay
            bounds={bounds}
            waypoints={waypoints}
            hideLabels={layersUnChecked['Waypoints']}
          />
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Dispensers" checked={!layersUnChecked['Dispensers']}>
          <WorkcellsOverlay
            bounds={bounds}
            workcells={dispensersData}
            hideLabels={layersUnChecked['Dispensers']}
            onWorkcellClick={onDispenserClick}
          />
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Ingestors" checked={!layersUnChecked['Ingestors']}>
          <WorkcellsOverlay
            bounds={bounds}
            workcells={ingestorsData}
            hideLabels={layersUnChecked['Ingestors']}
            onWorkcellClick={onIngestorClick}
          />
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Lifts" checked={!layersUnChecked['Lifts']}>
          <LiftsOverlay
            bounds={bounds}
            currentLevel={currentLevel.name}
            lifts={buildingMap.lifts}
            liftStates={liftStates}
            hideLabels={layersUnChecked['Lifts']}
            onLiftClick={onLiftClick}
          />
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Doors" checked={!layersUnChecked['Doors']}>
          <DoorsOverlay
            bounds={bounds}
            doors={currentLevel.doors}
            doorStates={doorStates}
            hideLabels={layersUnChecked['Doors']}
            onDoorClick={onDoorClick}
          />
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Trajectories" checked>
          <TrajectoriesOverlay bounds={bounds} trajectoriesData={renderedTrajectories} />
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Robots" checked={!layersUnChecked['Robots']}>
          <RobotsOverlay
            bounds={bounds}
            robots={robots}
            getRobotState={(fleet, robot) => {
              const getFleet = fleetStates[fleet];
              let state: RobotState = {};
              getFleet.robots ? (state = getFleet.robots[robot]) : (state = {});
              return state || null;
            }}
            hideLabels={layersUnChecked['Robots']}
            onRobotClick={onRobotClick}
          />
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
});
