import * as RmfModels from 'rmf-models';

export interface BaseRobotMarkerProps extends Omit<React.SVGAttributes<SVGGElement>, 'onClick'> {
  fleet: string;
  name: string;
  model: string;
  footprint: number;
  state: RmfModels.RobotState;
  inConflict?: boolean;
  /**
   * Whether the component should perform a translate transform to put it inline with the position
   * in RMF.
   *
   * default: true
   */
  translate?: boolean;
  onClick?(event: React.MouseEvent, fleet: string, robot: string): void;
}
