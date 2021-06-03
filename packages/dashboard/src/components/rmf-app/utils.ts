import * as RmfModels from 'rmf-models';
import { Place } from './place';

export function getPlaces(buildingMap: RmfModels.BuildingMap): Place[] {
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
