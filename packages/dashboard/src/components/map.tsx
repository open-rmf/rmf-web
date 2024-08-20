import { Box, styled, Typography, useMediaQuery } from '@mui/material';
import { Line } from '@react-three/drei';
import { Canvas, useLoader } from '@react-three/fiber';
import { BuildingMap, FleetState, Level, Lift } from 'api-client';
import Debug from 'debug';
import React, { ChangeEvent, Suspense } from 'react';
import {
  ColorManager,
  findSceneBoundingBoxFromThreeFiber,
  getPlaces,
  Place,
  ReactThreeFiberImageMaker,
  RobotData,
  RobotTableData,
  ShapeThreeRendering,
  TextThreeRendering,
} from 'react-components';
import { ErrorBoundary } from 'react-error-boundary';
import { Door as DoorModel } from 'rmf-models/ros/rmf_building_map_msgs/msg';
import { EMPTY, merge, scan, Subscription, switchMap, throttleTime } from 'rxjs';
import { Box3, TextureLoader, Vector3 } from 'three';

import { useAppController } from '../hooks/use-app-controller';
import { useAuthenticator } from '../hooks/use-authenticator';
import { FleetResource, useResources } from '../hooks/use-resources';
import { useRmfApi } from '../hooks/use-rmf-api';
import { TrajectoryData } from '../services/robot-trajectory-manager';
import { AppEvents } from './app-events';
import { DoorSummary } from './door-summary';
import { LiftSummary } from './lift-summary';
import { RobotSummary } from './robots/robot-summary';
import { CameraControl, Door, LayersController, Lifts, RobotThree } from './three-fiber';

const debug = Debug('MapApp');

const TrajectoryUpdateInterval = 2000;
// schedule visualizer manages it's own settings so that it doesn't cause a re-render
// of the whole app when it changes.
const colorManager = new ColorManager();

const DEFAULT_ROBOT_SCALE = 0.003;

function getRobotId(fleetName: string, robotName: string): string {
  return `${fleetName}/${robotName}`;
}

export interface MapProps {
  defaultMapLevel: string;
  defaultZoom: number;
  defaultRobotZoom: number;
  attributionPrefix: string;
}

export const Map = styled((props: MapProps) => {
  const authenticator = useAuthenticator();
  const { fleets: fleetResources } = useResources();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const rmfApi = useRmfApi();
  const { showAlert } = useAppController();
  const [currentLevel, setCurrentLevel] = React.useState<Level | undefined>(undefined);
  const [disabledLayers, setDisabledLayers] = React.useState<Record<string, boolean>>({
    'Pickup & Dropoff waypoints': false,
    'Pickup & Dropoff labels': true,
    Waypoints: true,
    'Waypoint labels': true,
    'Doors & Lifts': false,
    'Doors labels': true,
    Robots: false,
    'Robots labels': true,
    Trajectories: false,
  });
  const [openRobotSummary, setOpenRobotSummary] = React.useState(false);
  const [openDoorSummary, setOpenDoorSummary] = React.useState(false);
  const [openLiftSummary, setOpenLiftSummary] = React.useState(false);
  const [selectedRobot, setSelectedRobot] = React.useState<RobotTableData>();
  const [selectedDoor, setSelectedDoor] = React.useState<DoorModel>();
  const [selectedLift, setSelectedLift] = React.useState<Lift>();

  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);

  const [fleets, setFleets] = React.useState<FleetState[]>([]);

  const [waypoints, setWaypoints] = React.useState<Place[]>([]);
  const [currentLevelOfRobots, setCurrentLevelOfRobots] = React.useState<{
    [key: string]: string;
  }>({});

  const [trajectories, setTrajectories] = React.useState<TrajectoryData[]>([]);
  const trajectoryTime = 300000;
  const trajectoryAnimScale = trajectoryTime / (0.9 * TrajectoryUpdateInterval);
  const trajManager = rmfApi?.trajectoryManager;
  React.useEffect(() => {
    if (!currentLevel) {
      return;
    }

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
      const trajectories = resp.values.map((v) => ({
        trajectory: v,
        color: 'green',
        conflict: flatConflicts.includes(v.id),
        animationScale: trajectoryAnimScale,
        loopAnimation: false,
      }));

      // Filter trajectory due to https://github.com/open-rmf/rmf_visualization/issues/65
      for (const t of trajectories) {
        if (t.trajectory.segments.length === 0) {
          continue;
        }

        const knot = t.trajectory.segments[0];
        if ((knot.x[0] < 1e-9 && knot.x[0] > -1e-9) || (knot.x[1] < 1e-9 && knot.x[1] > -1e-9)) {
          t.trajectory.segments.shift();
        }
      }
      setTrajectories(trajectories);
    };

    updateTrajectory();
    const interval = window.setInterval(updateTrajectory, TrajectoryUpdateInterval);
    debug(`created trajectory update interval ${interval}`);

    return () => {
      cancel = true;
      clearInterval(interval);
      debug(`cleared interval ${interval}`);
    };
  }, [trajManager, currentLevel, trajectoryTime, trajectoryAnimScale, authenticator.token]);

  React.useEffect(() => {
    const levelByName = (map: BuildingMap, levelName?: string) => {
      if (!levelName) {
        return null;
      }
      for (const l of map.levels) {
        if (l.name === levelName) {
          return l;
        }
      }
      return null;
    };

    const handleBuildingMap = (newMap: BuildingMap) => {
      setBuildingMap(newMap);
      const loggedInDisplayLevel = AppEvents.justLoggedIn.value
        ? levelByName(newMap, props.defaultMapLevel)
        : undefined;
      const currentLevel = loggedInDisplayLevel || AppEvents.levelSelect.value || newMap.levels[0];
      AppEvents.levelSelect.next(currentLevel);
      setWaypoints(
        getPlaces(newMap).filter((p) => p.level === currentLevel.name && p.vertex.name.length > 0),
      );
      AppEvents.justLoggedIn.next(false);
    };

    (async () => {
      try {
        const newMap = (await rmfApi.buildingApi.getBuildingMapBuildingMapGet()).data;
        handleBuildingMap(newMap);
      } catch (e) {
        console.log(`failed to get building map: ${(e as Error).message}`);
      }
    })();

    const subs: Subscription[] = [];
    subs.push(rmfApi.buildingMapObs.subscribe((newMap) => handleBuildingMap(newMap)));
    subs.push(rmfApi.fleetsObs.subscribe(setFleets));

    return () => {
      for (const sub of subs) {
        sub.unsubscribe();
      }
    };
  }, [rmfApi, props.defaultMapLevel]);

  const [imageUrl, setImageUrl] = React.useState<string | null>(null);
  // Since the configurable zoom level is for supporting the lowest resolution
  // settings, we will double it for anything that is operating within modern
  // resolution settings.
  const defaultZoom = isScreenHeightLessThan800 ? props.defaultZoom : props.defaultZoom * 2;
  const [zoom, setZoom] = React.useState<number>(defaultZoom);
  const [sceneBoundingBox, setSceneBoundingBox] = React.useState<Box3 | undefined>(undefined);
  const [distance, setDistance] = React.useState<number>(0);

  React.useEffect(() => {
    const subs: Subscription[] = [];
    subs.push(
      AppEvents.zoom.subscribe((currentValue) => {
        setZoom(currentValue || defaultZoom);
      }),
    );
    subs.push(
      AppEvents.levelSelect.subscribe((currentValue) => {
        const newSceneBoundingBox = currentValue
          ? findSceneBoundingBoxFromThreeFiber(currentValue)
          : undefined;
        if (newSceneBoundingBox) {
          const center = newSceneBoundingBox.getCenter(new Vector3());
          const size = newSceneBoundingBox.getSize(new Vector3());
          const distance = Math.max(size.x, size.y, size.z) * 0.7;
          const newZoom = defaultZoom;
          AppEvents.resetCamera.next([center.x, center.y, center.z + distance, newZoom]);
        }
        setCurrentLevel(currentValue ?? undefined);
        setSceneBoundingBox(newSceneBoundingBox);
      }),
    );
    return () => {
      for (const sub of subs) {
        sub.unsubscribe();
      }
    };
  }, [defaultZoom]);

  React.useEffect(() => {
    if (!currentLevel?.images[0]) {
      setImageUrl(null);
      return;
    }

    (async () => {
      useLoader.preload(TextureLoader, currentLevel.images[0].data);
      setImageUrl(currentLevel.images[0].data);
    })();

    buildingMap &&
      setWaypoints(
        getPlaces(buildingMap).filter(
          (p) => p.level === currentLevel.name && p.vertex.name.length > 0,
        ),
      );
  }, [buildingMap, currentLevel]);

  const [robots, setRobots] = React.useState<RobotData[]>([]);
  const { current: robotsStore } = React.useRef<Record<string, RobotData>>({});
  React.useEffect(() => {
    (async () => {
      if (!currentLevel) {
        return;
      }
      const promises = Object.values(fleets).flatMap((fleetState) => {
        if (!fleetState.name || !fleetState.robots) {
          return null;
        }
        const robotKey = Object.keys(fleetState.robots);
        const fleetName = fleetState.name;
        return robotKey.map(async (r) => {
          const robotId = getRobotId(fleetName, r);
          const fleetResource: FleetResource | undefined = fleetResources[fleetName];
          if (robotId in robotsStore) return;
          robotsStore[robotId] = {
            fleet: fleetName,
            name: r,
            // no model name
            model: '',
            scale: fleetResource?.default.scale || DEFAULT_ROBOT_SCALE,
            footprint: 0.5,
            color: await colorManager.robotPrimaryColor(fleetName, r, ''),
            iconPath: fleetResource?.default.icon || undefined,
          };
        });
      });
      await Promise.all(promises);
      const newRobots = Object.values(fleets).flatMap((fleetState) => {
        const robotKey = fleetState.robots ? Object.keys(fleetState.robots) : [];
        return robotKey
          ?.filter(
            (r) =>
              fleetState.robots &&
              r in currentLevelOfRobots &&
              currentLevelOfRobots[r] === currentLevel.name &&
              `${fleetState.name}/${r}` in robotsStore,
          )
          .map((r) => robotsStore[`${fleetState.name}/${r}`]);
      });
      setRobots(newRobots);
    })();
  }, [fleets, fleetResources, robotsStore, currentLevel, currentLevelOfRobots]);

  const { current: robotLocations } = React.useRef<
    Record<string, [number, number, number, string]>
  >({});
  // updates the robot location
  React.useEffect(() => {
    const sub = rmfApi.fleetsObs
      .pipe(
        switchMap((fleets) =>
          merge(
            ...fleets.map((f) =>
              f.name
                ? rmfApi
                    .getFleetStateObs(f.name)
                    .pipe(throttleTime(500, undefined, { leading: true, trailing: true }))
                : EMPTY,
            ),
          ),
        ),
      )
      .subscribe((fleetState) => {
        const fleetName = fleetState.name;
        if (!fleetName || !fleetState.robots) {
          console.warn('Map: Fail to update robot location (missing fleet name or robots)');
          return;
        }
        Object.entries(fleetState.robots).forEach(([robotName, robotState]) => {
          const robotId = getRobotId(fleetName, robotName);
          if (!robotState.location) {
            console.warn(`Map: Fail to update robot location for ${robotId} (missing location)`);
            return;
          }
          robotLocations[robotId] = [
            robotState.location.x,
            robotState.location.y,
            robotState.location.yaw,
            robotState.location.map,
          ];

          setCurrentLevelOfRobots((prevState) => {
            if (!robotState.location?.map && prevState.robotName) {
              console.warn(`Map: Fail to update robot level for ${robotId} (missing map)`);
              const updatedState = { ...prevState };
              delete updatedState[robotName];
              return updatedState;
            }

            return {
              ...prevState,
              [robotName]: robotState.location?.map || '',
            };
          });
        });
      });
    return () => sub.unsubscribe();
  }, [rmfApi, robotLocations]);

  //Accumulate values over time to persist between tabs
  React.useEffect(() => {
    const sub = AppEvents.disabledLayers
      .pipe(scan((acc, value) => ({ ...acc, ...value }), {}))
      .subscribe((layers) => {
        setDisabledLayers(layers);
      });
    return () => sub.unsubscribe();
  }, []);

  // zoom to robot on select
  React.useEffect(() => {
    const subs: Subscription[] = [];

    // Centering on robot
    subs.push(
      AppEvents.robotSelect.subscribe((data) => {
        if (!data || !sceneBoundingBox) {
          return;
        }
        const [fleetName, robotName] = data;
        const robotId = getRobotId(fleetName, robotName);
        const robotLocation = robotLocations[robotId];
        if (!robotLocation) {
          console.warn(`Map: Failed to zoom to robot ${robotId} (robot location was not found)`);
          return;
        }

        const mapName = robotLocation[3];
        let newSceneBoundingBox = sceneBoundingBox;
        if (
          AppEvents.levelSelect.value &&
          AppEvents.levelSelect.value.name !== mapName &&
          buildingMap
        ) {
          const robotLevel =
            buildingMap.levels.find((l: Level) => l.name === mapName) || buildingMap.levels[0];
          AppEvents.levelSelect.next(robotLevel);

          const robotLevelSceneBoundingBox = findSceneBoundingBoxFromThreeFiber(robotLevel);
          if (!robotLevelSceneBoundingBox) {
            return;
          }
          newSceneBoundingBox = robotLevelSceneBoundingBox;
          setSceneBoundingBox(newSceneBoundingBox);
        }

        const size = newSceneBoundingBox.getSize(new Vector3());
        const distance = Math.max(size.x, size.y, size.z) * 0.7;
        const newZoom = props.defaultRobotZoom;
        AppEvents.resetCamera.next([
          robotLocation[0],
          robotLocation[1],
          robotLocation[2] + distance,
          newZoom,
        ]);
      }),
    );

    // Centering on door
    subs.push(
      AppEvents.doorSelect.subscribe((door) => {
        if (!door || !sceneBoundingBox) {
          return;
        }

        const [mapName, doorInfo] = door;

        let newSceneBoundingBox = sceneBoundingBox;
        if (
          AppEvents.levelSelect.value &&
          AppEvents.levelSelect.value.name !== mapName &&
          buildingMap
        ) {
          const doorLevel =
            buildingMap.levels.find((l: Level) => l.name === mapName) || buildingMap.levels[0];
          AppEvents.levelSelect.next(doorLevel);

          const doorLevelSceneBoundingBox = findSceneBoundingBoxFromThreeFiber(doorLevel);
          if (!doorLevelSceneBoundingBox) {
            return;
          }
          newSceneBoundingBox = doorLevelSceneBoundingBox;
          setSceneBoundingBox(newSceneBoundingBox);
        }

        const size = newSceneBoundingBox.getSize(new Vector3());
        const distance = Math.max(size.x, size.y, size.z) * 0.7;
        const newZoom = props.defaultRobotZoom;
        AppEvents.resetCamera.next([
          (doorInfo.v1_x + doorInfo.v2_x) / 2,
          (doorInfo.v1_y + doorInfo.v2_y) / 2,
          distance,
          newZoom,
        ]);
      }),
    );

    // Centering on lift
    subs.push(
      AppEvents.liftSelect.subscribe((lift) => {
        if (!lift || !sceneBoundingBox) {
          return;
        }

        const size = sceneBoundingBox.getSize(new Vector3());
        const distance = Math.max(size.x, size.y, size.z) * 0.7;
        const newZoom = props.defaultRobotZoom;
        AppEvents.resetCamera.next([lift.ref_x, lift.ref_y, distance, newZoom]);
      }),
    );

    return () => {
      for (const sub of subs) {
        sub.unsubscribe();
      }
    };
  }, [robotLocations, sceneBoundingBox, buildingMap, props.defaultRobotZoom]);

  React.useEffect(() => {
    if (!sceneBoundingBox) {
      return;
    }

    const size = sceneBoundingBox.getSize(new Vector3());
    setDistance(Math.max(size.x, size.y, size.z) * 0.7);
  }, [sceneBoundingBox]);

  return buildingMap && currentLevel && robotLocations ? (
    <Suspense fallback={null}>
      <LayersController
        disabledLayers={disabledLayers}
        levels={buildingMap.levels}
        currentLevel={currentLevel}
        onChange={(_event: ChangeEvent<HTMLInputElement>, value: string) => {
          AppEvents.levelSelect.next(
            buildingMap.levels.find((l: Level) => l.name === value) || buildingMap.levels[0],
          );
        }}
        handleFullView={() => {
          if (!sceneBoundingBox) {
            return;
          }
          const center = sceneBoundingBox.getCenter(new Vector3());
          const size = sceneBoundingBox.getSize(new Vector3());
          const distance = Math.max(size.x, size.y, size.z) * 0.7;
          const newZoom = defaultZoom;
          AppEvents.resetCamera.next([center.x, center.y, center.z + distance, newZoom]);
        }}
        handleZoomIn={() => AppEvents.zoomIn.next()}
        handleZoomOut={() => AppEvents.zoomOut.next()}
      />
      <Box
        component="div"
        sx={{
          position: 'absolute',
          bottom: '20px',
          right: '20px',
          width: 'auto',
          height: 'auto',
          zIndex: '1',
        }}
      >
        <Typography variant="caption" display="block">
          {props.attributionPrefix}
        </Typography>
      </Box>
      <Canvas
        onCreated={({ camera }) => {
          if (!sceneBoundingBox) {
            return;
          }
          const center = sceneBoundingBox.getCenter(new Vector3());
          camera.position.set(center.x, center.y, center.z + distance);
          camera.zoom = zoom;
          camera.updateProjectionMatrix();
        }}
        orthographic={true}
      >
        <CameraControl zoom={zoom} />
        {!disabledLayers['Pickup & Dropoff waypoints'] &&
          waypoints
            .filter((waypoint) => waypoint.pickupHandler || waypoint.dropoffHandler)
            .map((place, index) => (
              <ShapeThreeRendering
                key={index}
                position={[place.vertex.x, place.vertex.y, 0]}
                color="yellow"
                text={place.vertex.name}
                circleShape={false}
              />
            ))}
        {!disabledLayers['Pickup & Dropoff labels'] &&
          waypoints
            .filter((waypoint) => waypoint.pickupHandler || waypoint.dropoffHandler)
            .map((place, index) => (
              <TextThreeRendering
                key={index}
                position={[place.vertex.x, place.vertex.y, 0]}
                text={place.vertex.name}
              />
            ))}
        {!disabledLayers['Waypoints'] &&
          waypoints
            .filter((waypoint) => !waypoint.pickupHandler && !waypoint.dropoffHandler)
            .map((place, index) => (
              <ShapeThreeRendering
                key={index}
                position={[place.vertex.x, place.vertex.y, 0]}
                color="yellow"
                text={place.vertex.name}
                circleShape={false}
              />
            ))}
        {!disabledLayers['Waypoint labels'] &&
          waypoints
            .filter((waypoint) => !waypoint.pickupHandler && !waypoint.dropoffHandler)
            .map((place, index) => (
              <TextThreeRendering
                key={index}
                position={[place.vertex.x, place.vertex.y, 0]}
                text={place.vertex.name}
              />
            ))}
        {buildingMap.lifts.length > 0
          ? buildingMap.lifts.map((lift) =>
              lift.doors.map((door, i) => (
                <React.Fragment key={`${door.name}${i}`}>
                  {!disabledLayers['Doors labels'] && (
                    <TextThreeRendering position={[door.v1_x, door.v1_y, 0]} text={door.name} />
                  )}
                  {!disabledLayers['Doors & Lifts'] && (
                    <Door
                      door={door}
                      opacity={0.1}
                      height={8}
                      elevation={currentLevel.elevation}
                      lift={lift}
                      onDoorClick={(_ev) => {
                        setOpenLiftSummary(true);
                        setSelectedLift(lift);
                      }}
                    />
                  )}
                </React.Fragment>
              )),
            )
          : null}
        {!disabledLayers['Doors & Lifts'] && buildingMap.lifts.length > 0
          ? buildingMap.lifts.map((lift) =>
              lift.doors.map(() => (
                <Lifts
                  key={lift.name}
                  lift={lift}
                  height={8}
                  elevation={currentLevel.elevation}
                  opacity={0.1}
                  onLiftClick={(_ev, lift) => {
                    setOpenLiftSummary(true);
                    setSelectedLift(lift);
                  }}
                />
              )),
            )
          : null}
        {currentLevel.doors.length > 0
          ? currentLevel.doors.map((door, i) => (
              <React.Fragment key={`${door.name}${i}`}>
                {!disabledLayers['Doors labels'] && (
                  <TextThreeRendering position={[door.v1_x, door.v1_y, 0]} text={door.name} />
                )}
                {!disabledLayers['Doors'] && (
                  <Door
                    door={door}
                    opacity={0.1}
                    height={8}
                    elevation={currentLevel.elevation}
                    onDoorClick={(_ev, door) => {
                      setOpenDoorSummary(true);
                      setSelectedDoor(door);
                    }}
                  />
                )}
              </React.Fragment>
            ))
          : null}
        {currentLevel.images.length > 0 && imageUrl && (
          <ErrorBoundary
            fallback={<></>}
            onError={(error, info) => {
              console.error(error);
              console.log(info);
              showAlert(
                'error',
                'Unable to retrieve building map images. Please ensure that the building map server is operational and without issues.',
                20000,
              );
            }}
          >
            <ReactThreeFiberImageMaker level={currentLevel} imageUrl={imageUrl} />
          </ErrorBoundary>
        )}
        {!disabledLayers['Robots'] &&
          robots.map((robot) => {
            const robotId = `${robot.fleet}/${robot.name}`;
            if (robotId in robotLocations) {
              const location = robotLocations[robotId];
              const position: [number, number, number] = [location[0], location[1], location[2]];
              return (
                <RobotThree
                  key={`${robot.name} ${robot.fleet}`}
                  robot={robot}
                  robotLocation={position}
                  onRobotClick={(_ev, robot) => {
                    setOpenRobotSummary(true);
                    setSelectedRobot(robot);
                  }}
                  robotLabel={!disabledLayers['Robots labels']}
                />
              );
            }
            return null;
          })}
        {!disabledLayers['Trajectories'] &&
          trajectories.map((trajData) => (
            <Line
              key={trajData.trajectory.id}
              points={trajData.trajectory.segments.map((seg) => new Vector3(seg.x[0], seg.x[1], 4))}
              color={trajData.color}
              linewidth={5}
            />
          ))}
        <ambientLight />
      </Canvas>
      {openRobotSummary && selectedRobot && (
        <RobotSummary robot={selectedRobot} onClose={() => setOpenRobotSummary(false)} />
      )}
      {openDoorSummary && selectedDoor && (
        <DoorSummary
          onClose={() => setOpenDoorSummary(false)}
          door={selectedDoor}
          level={currentLevel}
        />
      )}

      {openLiftSummary && selectedLift && (
        <LiftSummary onClose={() => setOpenLiftSummary(false)} lift={selectedLift} />
      )}
    </Suspense>
  ) : null;
})({
  // This ensures that the resize handle is above the map.
  '& > .react-resizable-handle': {
    zIndex: 1001,
  },
});

export default Map;
