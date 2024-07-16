/* This is a generated file, do not edit */

import * as rmf_building_map_msgs from '../../rmf_building_map_msgs';

export class GetBuildingMap_Response {
  static readonly FullTypeName = '';

  building_map: rmf_building_map_msgs.msg.BuildingMap;

  constructor(fields: Partial<GetBuildingMap_Response> = {}) {
    this.building_map = fields.building_map || new rmf_building_map_msgs.msg.BuildingMap();
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      rmf_building_map_msgs.msg.BuildingMap.validate(
        obj['building_map'] as Record<string, unknown>,
      );
    } catch (e) {
      throw new Error('in "building_map":\n  ' + (e as Error).message);
    }
  }
}

export default GetBuildingMap_Response;
