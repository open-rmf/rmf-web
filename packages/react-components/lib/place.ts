import type { BuildingMap, GraphNode } from 'api-client';

const DEFAULT_PICKUP_POINT_PARAM_NAME = 'pickup_dispenser';
const DEFAULT_DROPOFF_POINT_PARAM_NAME = 'dropoff_ingestor';
const DEFAULT_CLEANING_ZONE_PARAM_NAME = 'is_cleaning_zone';

export interface Place {
  level: string;
  vertex: GraphNode;
}

export interface NamedPlaces {
  places: Place[];
  pickupPoints: Place[];
  dropoffPoints: Place[];
  cleaningZones: Place[];
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

export function getNamedPlaces(buildingMap: BuildingMap): NamedPlaces {
  const places = new Map<string, Place>();
  const pickupPoints = new Map<string, Place>();
  const dropoffPoints = new Map<string, Place>();
  const cleaningZones = new Map<string, Place>();

  for (const level of buildingMap.levels) {
    for (const graphs of level.nav_graphs) {
      for (const vertex of graphs.vertices) {
        if (vertex.name) {
          const place: Place = { level: level.name, vertex };
          for (const p of vertex.params) {
            if (p.name === DEFAULT_PICKUP_POINT_PARAM_NAME) {
              pickupPoints.set(vertex.name, place);
            }
            if (p.name === DEFAULT_DROPOFF_POINT_PARAM_NAME) {
              dropoffPoints.set(vertex.name, place);
            }
            if (p.name === DEFAULT_CLEANING_ZONE_PARAM_NAME) {
              cleaningZones.set(vertex.name, place);
            }
          }
          places.set(vertex.name, place);
        }
      }
    }
  }
  return {
    places: Array.from(places.values()),
    pickupPoints: Array.from(pickupPoints.values()),
    dropoffPoints: Array.from(dropoffPoints.values()),
    cleaningZones: Array.from(cleaningZones.values()),
  };
}
