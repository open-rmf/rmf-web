import type { BuildingMap, GraphNode } from 'api-client';

const DEFAULT_PICKUP_WAYPOINT_PARAM_NAME = 'pickup_dispenser';
const DEFAULT_DROPOFF_WAYPOINT_PARAM_NAME = 'dropoff_ingestor';
const DEFAULT_CLEANING_WAYPOINT_PARAM_NAME = 'is_cleaning_zone';

export interface Waypoint {
  level: string;
  vertex: GraphNode;
}

export interface WaypointCollection {
  waypoints: Waypoint[];
  pickupWaypoints: Waypoint[];
  dropoffWaypoints: Waypoint[];
  cleaningWaypoints: Waypoint[];
}

export function getWaypoints(buildingMap: BuildingMap): Waypoint[] {
  const waypoints: Waypoint[] = [];
  for (const level of buildingMap.levels) {
    for (const graphs of level.nav_graphs) {
      for (const vertex of graphs.vertices) {
        if (vertex.name) {
          waypoints.push({ level: level.name, vertex });
        }
      }
    }
  }
  return waypoints;
}

export function getWaypointCollection(buildingMap: BuildingMap): WaypointCollection {
  const waypoints = new Map<string, Waypoint>();
  const pickupWaypoints = new Map<string, Waypoint>();
  const dropoffWaypoints = new Map<string, Waypoint>();
  const cleaningWaypoints = new Map<string, Waypoint>();

  for (const level of buildingMap.levels) {
    for (const graphs of level.nav_graphs) {
      for (const vertex of graphs.vertices) {
        if (vertex.name) {
          const waypoint: Waypoint = { level: level.name, vertex };
          for (const p of vertex.params) {
            if (p.name === DEFAULT_PICKUP_WAYPOINT_PARAM_NAME) {
              pickupWaypoints.set(vertex.name, waypoint);
            }
            if (p.name === DEFAULT_DROPOFF_WAYPOINT_PARAM_NAME) {
              dropoffWaypoints.set(vertex.name, waypoint);
            }
            if (p.name === DEFAULT_CLEANING_WAYPOINT_PARAM_NAME) {
              cleaningWaypoints.set(vertex.name, waypoint);
            }
          }
          waypoints.set(vertex.name, waypoint);
        }
      }
    }
  }
  return {
    waypoints: Array.from(waypoints.values()),
    pickupWaypoints: Array.from(pickupWaypoints.values()),
    dropoffWaypoints: Array.from(dropoffWaypoints.values()),
    cleaningWaypoints: Array.from(cleaningWaypoints.values()),
  };
}
