/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import { Dispenser, Ingestor } from 'api-client';
import Debug from 'debug';
import * as L from 'leaflet';
import React from 'react';
import {
  affineImageBounds,
  AffineImageOverlay,
  ColorManager,
  DoorsOverlay as DoorsOverlay_,
  getPlaces,
  LiftsOverlay as LiftsOverlay_,
  LMap,
  loadAffineImage,
  RobotData,
  RobotsOverlay as RobotsOverlay_,
  RobotsOverlayProps,
  TrajectoriesOverlay as TrajectoriesOverlay_,
  TrajectoryData,
  TrajectoryTimeControl,
  useAsync,
  WaypointsOverlay as WaypointsOverlay_,
  WorkcellData,
  WorkcellsOverlay as WorkcellsOverlay_,
} from 'react-components';
import { AttributionControl, LayersControl } from 'react-leaflet';
import * as RmfModels from 'rmf-models';
import { Subscription, throttleTime } from 'rxjs';
import appConfig from '../../app-config';
import { NegotiationTrajectoryResponse } from '../../managers/negotiation-status-manager';
import { ResourcesContext } from '../app-contexts';
import { RmfIngressContext, RxRmfContext } from '../rmf-app';

const DoorsOverlay = React.memo(DoorsOverlay_);
const LiftsOverlay = React.memo(LiftsOverlay_);
const RobotsOverlay = React.memo(RobotsOverlay_);
const TrajectoriesOverlay = React.memo(TrajectoriesOverlay_);
const WaypointsOverlay = React.memo(WaypointsOverlay_);
const WorkcellsOverlay = React.memo(WorkcellsOverlay_);

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

export function ScheduleVisualizer({
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
  const rmfIngress = React.useContext(RmfIngressContext);
  const rxRmf = React.useContext(RxRmfContext);

  const [buildingMap, setBuildingMap] = React.useState<RmfModels.BuildingMap | null>(null);
  React.useEffect(() => {
    if (!rmfIngress) return;
    (async () => {
      const buildingMap = (await safeAsync(rmfIngress.buildingApi.getBuildingMapBuildingMapGet()))
        .data as RmfModels.BuildingMap;
      setBuildingMap(buildingMap);
    })();
  }, [safeAsync, rmfIngress]);

  const classes = scheduleVisualizerStyle();
  const levels = React.useMemo(
    () => (buildingMap ? [...buildingMap.levels].sort((a, b) => a.name.localeCompare(b.name)) : []),
    [buildingMap],
  );
  const [currentLevel, setCurrentLevel] = React.useState<RmfModels.Level | null>(null);
  React.useEffect(() => {
    if (levels.length === 0) return;
    setCurrentLevel(levels[0]);
  }, [levels]);

  const [images, setImages] = React.useState<Record<string, HTMLImageElement>>({});
  const [levelBounds, setLevelBounds] = React.useState<Record<string, L.LatLngBoundsExpression>>(
    {},
  );
  const bounds = React.useMemo(
    () => (currentLevel !== null ? levelBounds[currentLevel.name] : null),
    [levelBounds, currentLevel],
  );

  const [doorStates, setDoorStates] = React.useState<Record<string, RmfModels.DoorState>>({});
  React.useEffect(() => {
    if (!rxRmf || !buildingMap) return;
    const doors = buildingMap.levels.flatMap((level) => level.doors);
    const subs = doors.map((door) =>
      rxRmf
        .doorStates(door.name)
        .pipe(throttleTime(1000))
        .subscribe(
          (doorState) =>
            doorState && setDoorStates((prev) => ({ ...prev, [door.name]: doorState })),
        ),
    );
    return () => {
      subs.forEach((sub) => sub.unsubscribe());
    };
  }, [rxRmf, buildingMap]);

  const [liftStates, setLiftStates] = React.useState<Record<string, RmfModels.LiftState>>({});
  React.useEffect(() => {
    if (!rxRmf || !buildingMap) return;
    const subs = buildingMap.lifts.map((lift) =>
      rxRmf
        .liftStates(lift.name)
        .pipe(throttleTime(1000))
        .subscribe(
          (liftState) =>
            liftState && setLiftStates((prev) => ({ ...prev, [lift.name]: liftState })),
        ),
    );
    return () => {
      subs.forEach((sub) => sub.unsubscribe());
    };
  }, [rxRmf, buildingMap]);

  // FIXME: trajectory manager should handle the tokens
  const authenticator = appConfig.authenticator;

  const [trajectories, setTrajectories] = React.useState<TrajectoryData[]>([]);
  const trajManager = rmfIngress ? rmfIngress.trajectoryManager : null;

  const [
    scheduleVisualizerSettings,
    setScheduleVisualizerSettings,
  ] = React.useState<ScheduleVisualizerSettings>(() => {
    const settings = window.localStorage.getItem(SettingsKey);
    return settings ? JSON.parse(settings) : { trajectoryTime: 60000 /* 1 min */ };
  });
  const trajectoryTime = scheduleVisualizerSettings.trajectoryTime;
  const trajectoryAnimScale = trajectoryTime / (0.9 * TrajectoryUpdateInterval);

  const { current: negotiationTrajStore } = React.useRef<
    Record<string, NegotiationTrajectoryResponse>
  >({});
  const negoTrajectories = React.useMemo<TrajectoryData[]>(() => {
    if (mode !== 'negotiation' || currentLevel === null) return [];
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
    if (mode !== 'normal' || currentLevel === null) return;

    let interval: number;
    let cancel = false;

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
    if (!rmfIngress || currentLevel === null) return;
    const dispenserManager = resourceManager?.dispensers;
    if (!dispenserManager) return;

    (async () => {
      const dispensers = (await safeAsync(rmfIngress.dispensersApi.getDispensersDispensersGet()))
        .data as Dispenser[];

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
  }, [safeAsync, rmfIngress, resourceManager?.dispensers, currentLevel]);

  const [ingestorsData, setIngestorsData] = React.useState<WorkcellData[]>([]);
  React.useEffect(() => {
    if (!rmfIngress || currentLevel === null) return;
    const dispenserManager = resourceManager?.dispensers;
    if (!dispenserManager) return;

    (async () => {
      const ingestors = (await safeAsync(rmfIngress.ingestorsApi.getIngestorsIngestorsGet()))
        .data as Ingestor[];

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
  }, [safeAsync, rmfIngress, resourceManager?.dispensers, currentLevel]);

  const waypoints = React.useMemo(() => {
    if (!buildingMap || currentLevel === null) return [];
    return getPlaces(buildingMap).filter(
      (p) => p.level === currentLevel.name && p.vertex.name.length > 0,
    );
  }, [buildingMap, currentLevel]);

  const [fleetStates, setFleetStates] = React.useState<Record<string, RmfModels.FleetState>>({});
  React.useEffect(() => {
    if (!rmfIngress || !rxRmf) return;
    let subs: Subscription[] = [];
    (async () => {
      const fleets = (await safeAsync(rmfIngress.fleetsApi.getFleetsFleetsGet())).data;
      subs = fleets.map((fleet) =>
        rxRmf
          .fleetStates(fleet.name)
          .pipe(throttleTime(1000))
          .subscribe(
            (fleetState) =>
              fleetState && setFleetStates((s) => ({ ...s, [fleet.name]: fleetState })),
          ),
      );
    })();
    return () => {
      subs.forEach((s) => s.unsubscribe());
    };
  }, [safeAsync, rmfIngress, rxRmf]);

  const [robotsRecord, setRobotsRecord] = React.useState<Record<string, RobotData>>({});
  React.useEffect(() => {
    Object.values(fleetStates).forEach(async (fleetState) => {
      if (!fleetState) return;
      fleetState.robots.map(async (r) => {
        const robotId = `${fleetState.name}/${r.name}`;
        const color = await safeAsync(
          colorManager.robotPrimaryColor(fleetState.name, r.name, r.model),
        );
        const iconPath =
          (await safeAsync(resourceManager?.robots.getIconPath(fleetState.name, r.model))) ||
          undefined;
        setRobotsRecord((prev) => {
          if (robotId in prev) return prev;
          return {
            ...prev,
            [robotId]: {
              fleet: fleetState.name,
              name: r.name,
              model: r.model,
              footprint: 0.5,
              color,
              iconPath,
            },
          };
        });
      });
    });
  }, [safeAsync, fleetStates, resourceManager?.robots]);
  const robots = React.useMemo(() => Object.values(robotsRecord), [robotsRecord]);

  const getRobotState = React.useCallback<RobotsOverlayProps['getRobotState']>(
    (fleet, robot) => {
      const state = fleetStates[fleet].robots.find((r) => r.name === robot);
      return state || null;
    },
    [fleetStates],
  );

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
    if (levels === null) return;
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
      }}
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
        {levels.map((level) => (
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
          {buildingMap && currentLevel && (
            <LiftsOverlay
              bounds={bounds}
              currentLevel={currentLevel.name}
              lifts={buildingMap.lifts}
              liftStates={liftStates}
              hideLabels={layersUnChecked['Lifts']}
              onLiftClick={onLiftClick}
            />
          )}
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Doors" checked={!layersUnChecked['Doors']}>
          {currentLevel && (
            <DoorsOverlay
              bounds={bounds}
              doors={currentLevel.doors}
              doorStates={doorStates}
              hideLabels={layersUnChecked['Doors']}
              onDoorClick={onDoorClick}
            />
          )}
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Trajectories" checked>
          <TrajectoriesOverlay bounds={bounds} trajectoriesData={renderedTrajectories} />
        </LayersControl.Overlay>

        <LayersControl.Overlay name="Robots" checked={!layersUnChecked['Robots']}>
          <RobotsOverlay
            bounds={bounds}
            robots={robots}
            getRobotState={getRobotState}
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
}

export default ScheduleVisualizer;
