import { createDeferredContext } from './deferred-context';

export interface RobotResource {
  /**
   * Path to an image to be used as the robot's icon.
   */
  icon?: string;

  /**
   * Scale of the image to match the robot's dimensions.
   */
  scale?: number;
}

export interface FleetResource {
  // TODO(koonpeng): configure robot resources based on robot model, this will require https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/robot_state.json to expose the robot model.
  // [robotModel: string]: RobotResource;
  default: RobotResource;
}

export interface LogoResource {
  /**
   * Path to an image to be used as the logo on the app bar.
   */
  header: string;
}

export interface Resources {
  fleets: { [fleetName: string]: FleetResource };
  logos: LogoResource;
}

export const [useResources, ResourcesProvider] = createDeferredContext<Resources>();
