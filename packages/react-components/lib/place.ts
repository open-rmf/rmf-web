import type { BuildingMap, GraphNode } from 'api-client';

const DEFAULT_PICKUP_POINT_PARAM_NAME = 'pickup_dispenser';
const DEFAULT_DROPOFF_POINT_PARAM_NAME = 'dropoff_ingestor';
const DEFAULT_CLEANING_ZONE_PARAM_NAME = 'is_cleaning_zone';

export interface Place {
  level: string;
  vertex: GraphNode;
  pickupHandler?: string;
  dropoffHandler?: string;
  cleaningZone?: boolean;
}

export function getPlaces(buildingMap: BuildingMap): Place[] {
  const places = new Map<string, Place>();
  for (const level of buildingMap.levels) {
    for (const graphs of level.nav_graphs) {
      for (const vertex of graphs.vertices) {
        if (!vertex.name) {
          continue;
        }
        const place: Place = { level: level.name, vertex };
        for (const p of vertex.params) {
          if (p.name === DEFAULT_PICKUP_POINT_PARAM_NAME) {
            place.pickupHandler = p.value_string;
          }
          if (p.name === DEFAULT_DROPOFF_POINT_PARAM_NAME) {
            place.dropoffHandler = p.value_string;
          }
          if (p.name === DEFAULT_CLEANING_ZONE_PARAM_NAME) {
            place.cleaningZone = true;
          }
        }
        places.set(vertex.name, place);
      }
    }
  }
  return Array.from(places.values());
}
