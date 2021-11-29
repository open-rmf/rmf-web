/* This is a generated file, do not edit */

import { BuildingMap } from '../../rmf_building_map_msgs/msg/BuildingMap';

export class GetBuildingMap_Response {
  static readonly FullTypeName = 'rmf_building_map_msgs/srv/GetBuildingMap_Response';

  building_map: BuildingMap;

  constructor(fields: Partial<GetBuildingMap_Response> = {}) {
    this.building_map = fields.building_map || new BuildingMap();
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      BuildingMap.validate(obj['building_map'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "building_map":\n  ' + (e as Error).message);
    }
  }
}

/*

BuildingMap building_map

*/
