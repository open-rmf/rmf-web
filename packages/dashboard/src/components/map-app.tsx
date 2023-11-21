import { styled } from '@mui/material';
import {
  BuildingMap,
  Dispenser,
  ApiServerModelsRmfApiFleetStateFleetState,
  Ingestor,
  Level,
} from 'api-client';
import Debug from 'debug';
import React, { ChangeEvent, Suspense } from 'react';
import {
  ColorManager,
  findSceneBoundingBoxFromThreeFiber,
  getPlaces,
  Place,
  ReactThreeFiberImageMaker,
  RobotTableData,
  ShapeThreeRendering,
  TextThreeRendering,
  RobotData,
} from 'react-components';
import { EMPTY, merge, scan, Subscription, switchMap } from 'rxjs';
import appConfig from '../app-config';
import { ResourcesContext } from './app-contexts';
import { AppEvents } from './app-events';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';
import { TrajectoryData } from './trajectories-overlay';
import { WorkcellData } from './workcells-overlay';
import { RobotSummary } from './robots/robot-summary';
import { Box3, TextureLoader, Vector3 } from 'three';
import { Canvas, useLoader } from '@react-three/fiber';
import { Line } from '@react-three/drei';
import { CameraControl, LayersController } from './three-fiber';
import { Lifts, Door, RobotThree } from './three-fiber';

type FleetState = ApiServerModelsRmfApiFleetStateFleetState;

const debug = Debug('MapApp');

const TrajectoryUpdateInterval = 2000;
// schedule visualizer manages it's own settings so that it doesn't cause a re-render
// of the whole app when it changes.
const colorManager = new ColorManager();

const DEFAULT_ZOOM_LEVEL = 20;
const DEFAULT_ROBOT_ZOOM_LEVEL = 6;
const DEFAULT_ROBOT_SCALE = 0.003;

function getRobotId(fleetName: string, robotName: string): string {
  return `${fleetName}/${robotName}`;
}

export const MapApp = styled(
  createMicroApp('Map', () => {
    const rmf = React.useContext(RmfAppContext);
    const resourceManager = React.useContext(ResourcesContext);
    const [currentLevel, setCurrentLevel] = React.useState<Level | undefined>(undefined);
    const [disabledLayers, setDisabledLayers] = React.useState<Record<string, boolean>>({
      Waypoints: false,
      Dispensers: false,
      Ingestors: false,
      Lifts: false,
      Doors: false,
      Trajectories: false,
      Robots: false,
      'Door and lift labels': true,
      'Waypoint labels': true,
      'Pickup point labels': true,
      'Dropoff point labels': false,
    });
    const [openRobotSummary, setOpenRobotSummary] = React.useState(false);
    const [selectedRobot, setSelectedRobot] = React.useState<RobotTableData>();

    const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);

    const [dispensers, setDispensers] = React.useState<Dispenser[]>([]);
    const [dispensersData, setDispensersData] = React.useState<WorkcellData[]>([]);
    React.useEffect(() => {
      if (!currentLevel) {
        return;
      }
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
        const icons = await Promise.all(promises);
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
    }, [resourceManager?.dispensers, dispensers, currentLevel]);

    const [ingestors, setIngestors] = React.useState<Ingestor[]>([]);
    const [ingestorsData, setIngestorsData] = React.useState<WorkcellData[]>([]);
    React.useEffect(() => {
      if (!currentLevel) {
        return;
      }
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
        const icons = await Promise.all(promises);
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
    }, [resourceManager?.dispensers, ingestors, currentLevel]);

    const [fleets, setFleets] = React.useState<FleetState[]>([]);

    const [waypoints, setWaypoints] = React.useState<Place[]>([]);
    const [currentLevelOfRobots, setCurrentLevelOfRobots] = React.useState<{
      [key: string]: string;
    }>({});

    const [trajectories, setTrajectories] = React.useState<TrajectoryData[]>([]);
    const trajectoryTime = 300000;
    const trajectoryAnimScale = trajectoryTime / (0.9 * TrajectoryUpdateInterval);
    const trajManager = rmf?.trajectoryManager;
    React.useEffect(() => {
      if (!currentLevel) {
        return;
      }

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
          token: appConfig.authenticator.token,
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
    }, [trajManager, currentLevel, trajectoryTime, trajectoryAnimScale]);

    React.useEffect(() => {
      if (!rmf) {
        return;
      }

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
          ? levelByName(newMap, resourceManager?.loggedInDisplayLevel)
          : undefined;
        const currentLevel =
          loggedInDisplayLevel || AppEvents.levelSelect.value || newMap.levels[0];
        AppEvents.levelSelect.next(currentLevel);
        setWaypoints(
          getPlaces(newMap).filter(
            (p) => p.level === currentLevel.name && p.vertex.name.length > 0,
          ),
        );
        AppEvents.justLoggedIn.next(false);
      };

      (async () => {
        try {
          const newMap = (await rmf.buildingApi.getBuildingMapBuildingMapGet()).data;
          handleBuildingMap(newMap);
        } catch (e) {
          console.log(`failed to get building map: ${(e as Error).message}`);
        }
      })();

      const subs: Subscription[] = [];
      subs.push(rmf.buildingMapObs.subscribe((newMap) => handleBuildingMap(newMap)));
      subs.push(rmf.dispensersObs.subscribe(setDispensers));
      subs.push(rmf.ingestorsObs.subscribe(setIngestors));
      subs.push(rmf.fleetsObs.subscribe(setFleets));

      return () => {
        for (const sub of subs) {
          sub.unsubscribe();
        }
      };
    }, [rmf, resourceManager]);

    const [imageUrl, setImageUrl] = React.useState<string | null>(null);
    const [zoom, setZoom] = React.useState<number>(
      resourceManager?.defaultZoom || DEFAULT_ZOOM_LEVEL,
    );
    const [sceneBoundingBox, setSceneBoundingBox] = React.useState<Box3 | undefined>(undefined);
    const [distance, setDistance] = React.useState<number>(0);

    React.useEffect(() => {
      const subs: Subscription[] = [];
      subs.push(
        AppEvents.zoom.subscribe((currentValue) => {
          setZoom(currentValue || resourceManager?.defaultZoom || DEFAULT_ZOOM_LEVEL);
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
            const newZoom = resourceManager?.defaultZoom || DEFAULT_ZOOM_LEVEL;
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
    }, [resourceManager?.defaultZoom]);

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
          const robotKey = fleetState.robots && Object.keys(fleetState.robots);
          const fleetName = fleetState.name ? fleetState.name : '';
          return robotKey?.map(async (r) => {
            const robotId = getRobotId(fleetName, r);
            if (robotId in robotsStore) return;
            robotsStore[robotId] = {
              fleet: fleetName,
              name: r,
              // no model name
              model: '',
              scale: resourceManager?.robots.getRobotIconScale(fleetName, r) || DEFAULT_ROBOT_SCALE,
              footprint: 0.5,
              color: await colorManager.robotPrimaryColor(fleetName, r, ''),
              iconPath: (await resourceManager?.robots.getIconPath(fleetName, r)) || undefined,
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
    }, [fleets, robotsStore, resourceManager, currentLevel, currentLevelOfRobots]);

    const { current: robotLocations } = React.useRef<
      Record<string, [number, number, number, string]>
    >({});
    // updates the robot location
    React.useEffect(() => {
      if (!rmf) {
        return;
      }
      const sub = rmf.fleetsObs
        .pipe(
          switchMap((fleets) =>
            merge(...fleets.map((f) => (f.name ? rmf.getFleetStateObs(f.name) : EMPTY))),
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
              if (!robotState.location?.map && prevState.hasOwnProperty(robotName)) {
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
    }, [rmf, robotLocations]);

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
      const sub = AppEvents.robotSelect.subscribe((data) => {
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
        const newZoom = resourceManager?.defaultRobotZoom ?? DEFAULT_ROBOT_ZOOM_LEVEL;
        AppEvents.resetCamera.next([
          robotLocation[0],
          robotLocation[1],
          robotLocation[2] + distance,
          newZoom,
        ]);
      });
      return () => sub.unsubscribe();
    }, [robotLocations, resourceManager?.defaultRobotZoom, sceneBoundingBox, buildingMap]);

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
          onChange={(event: ChangeEvent<HTMLInputElement>, value: string) => {
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
            const newZoom = resourceManager?.defaultZoom || DEFAULT_ZOOM_LEVEL;
            AppEvents.resetCamera.next([center.x, center.y, center.z + distance, newZoom]);
          }}
          handleZoomIn={() => AppEvents.zoomIn.next()}
          handleZoomOut={() => AppEvents.zoomOut.next()}
        />
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
          {currentLevel.doors.length > 0
            ? currentLevel.doors.map((door, i) => (
                <React.Fragment key={`${door.name}${i}`}>
                  {!disabledLayers['Door and lift labels'] && (
                    <TextThreeRendering position={[door.v1_x, door.v1_y, 0]} text={door.name} />
                  )}
                  {!disabledLayers['Doors'] && (
                    <Door door={door} opacity={0.1} height={8} elevation={currentLevel.elevation} />
                  )}
                </React.Fragment>
              ))
            : null}
          {currentLevel.images.length > 0 && imageUrl && (
            <ReactThreeFiberImageMaker level={currentLevel} imageUrl={imageUrl} />
          )}
          {buildingMap.lifts.length > 0
            ? buildingMap.lifts.map((lift, i) =>
                lift.doors.map((door, i) => (
                  <React.Fragment key={`${door.name}${i}`}>
                    {!disabledLayers['Door and lift labels'] && (
                      <TextThreeRendering position={[door.v1_x, door.v1_y, 0]} text={door.name} />
                    )}
                    {!disabledLayers['Doors'] && (
                      <Door
                        door={door}
                        opacity={0.1}
                        height={8}
                        elevation={currentLevel.elevation}
                        lift={lift}
                      />
                    )}
                  </React.Fragment>
                )),
              )
            : null}

          {!disabledLayers['Lifts'] && buildingMap.lifts.length > 0
            ? buildingMap.lifts.map((lift) =>
                lift.doors.map(() => (
                  <Lifts
                    key={lift.name}
                    lift={lift}
                    height={8}
                    elevation={currentLevel.elevation}
                    opacity={0.1}
                  />
                )),
              )
            : null}

          {!disabledLayers['Waypoints'] &&
            waypoints.map((place, index) => (
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
          {!disabledLayers['Pickup point labels'] &&
            waypoints
              .filter((waypoint) => waypoint.pickupHandler)
              .map((place, index) => (
                <TextThreeRendering
                  key={index}
                  position={[place.vertex.x, place.vertex.y, 0]}
                  text={place.vertex.name}
                />
              ))}
          {!disabledLayers['Dropoff point labels'] &&
            waypoints
              .filter((waypoint) => waypoint.dropoffHandler)
              .map((place, index) => (
                <TextThreeRendering
                  key={index}
                  position={[place.vertex.x, place.vertex.y, 0]}
                  text={place.vertex.name}
                />
              ))}
          {!disabledLayers['Ingestors'] &&
            ingestorsData.map((ingestor, index) => (
              <ShapeThreeRendering
                key={index}
                position={[ingestor.location[0], ingestor.location[1], 0]}
                color="red"
                circleShape={true}
              />
            ))}

          {!disabledLayers['Dispensers'] &&
            dispensersData.map((dispenser, index) => (
              <ShapeThreeRendering
                key={index}
                position={[dispenser.location[0], dispenser.location[1], 0]}
                color="red"
                circleShape={true}
              />
            ))}
          {!disabledLayers['Trajectories'] &&
            trajectories.map((trajData) => (
              <Line
                key={trajData.trajectory.id}
                points={trajData.trajectory.segments.map(
                  (seg) => new Vector3(seg.x[0], seg.x[1], 4),
                )}
                color={trajData.color}
                linewidth={5}
              />
            ))}
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
                  />
                );
              }
              return null;
            })}
          <ambientLight />
        </Canvas>
        {openRobotSummary && selectedRobot && (
          <RobotSummary robot={selectedRobot} onClose={() => setOpenRobotSummary(false)} />
        )}
      </Suspense>
    ) : null;
  }),
)({
  // This ensures that the resize handle is above the map.
  '& > .react-resizable-handle': {
    zIndex: 1001,
  },
});
