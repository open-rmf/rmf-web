import * as RmfModels from 'rmf-models';

export function getPlaces(buildingMap: RmfModels.BuildingMap): Record<string, RmfModels.GraphNode> {
  const navGraphs = buildingMap.levels.flatMap((level) => level.nav_graphs);
  const vertices = navGraphs.flatMap((graph) => graph.vertices);
  return vertices.reduce<Record<string, RmfModels.GraphNode>>((obj, v) => {
    if (v.name) {
      obj[v.name] = v;
    }
    return obj;
  }, {});
}
