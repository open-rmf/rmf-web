import type { BuildingMap, GraphNode } from 'api-client';

export interface Waypoint {
  level: string;
  vertex: GraphNode;
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
