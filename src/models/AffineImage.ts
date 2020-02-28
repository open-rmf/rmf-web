import { IPose2D } from './Pose2D';

export interface IAffineImage {
  name: string;
  scale: number;
  encoding: string;
  data: Uint8Array;
  pose: IPose2D;
}