import type { BuildingMap, GraphNode } from 'api-client';

export interface Place {
  level: string;
  vertex: GraphNode;
}

export function getPlaces(buildingMap: BuildingMap): Place[] {
  const places: Place[] = [];
  for (const level of buildingMap.levels) {
    for (const graphs of level.nav_graphs) {
      for (const vertex of graphs.vertices) {
        if (vertex.name) {
          places.push({ level: level.name, vertex });
        }
      }
    }
  }
  return places;
}
