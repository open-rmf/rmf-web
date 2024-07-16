import {
  AffineImage as RmfAffineImage,
  BuildingMap as RmfBuildingMap,
  Level as RmfLevel,
} from './ros/rmf_building_map_msgs/msg';

export * from './ros';

export interface AffineImage extends Omit<RmfAffineImage, 'data'> {
  data: string;
}

export interface Level extends Omit<RmfLevel, 'images'> {
  images: AffineImage[];
}

export interface BuildingMap extends Omit<RmfBuildingMap, 'levels'> {
  levels: Level[];
}

export * from './version';
