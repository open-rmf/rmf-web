import Debug from 'debug';
import { DispenserResourceManager, RawDispenserResource } from './resource-manager-dispensers';
import { LogoResource, LogoResourceManager } from './resource-manager-logos';
import { RobotResource, RobotResourceManager } from './resource-manager-robots';

const debug = Debug('ResourceManager');
const ResourceFile = 'resources/main.json';

export interface ResourceConfigurationsType {
  robots?: Record<string, RobotResource>; // Record<FleetName, RobotResource>
  dispensers?: Record<string, RawDispenserResource>; // Record<DispenserName, DispenserResource>
  logos?: Record<string, LogoResource>;
  helpLink?: string;
  reportIssue?: string;
}

export default class ResourceManager {
  robots: RobotResourceManager;
  logos: LogoResourceManager;
  dispensers?: DispenserResourceManager;
  helpLink: string;
  reportIssue: string;

  /**
   * Gets the default resource manager using the embedded resource file (aka "assets/resources/main.json").
   */
  static defaultResourceManager = async (): Promise<ResourceManager | undefined> => {
    try {
      // need to use interpolate string to make webpack resolve import at run time and for
      // typescript to not attempt to typecheck it.
      const resources = (await import(
        /* webpackMode: "eager" */ `../assets/${ResourceFile}`
      )) as ResourceConfigurationsType;
      return new ResourceManager(resources);
    } catch {
      debug('failed to load resource file');
      return new ResourceManager({});
    }
  };

  constructor(resources: ResourceConfigurationsType) {
    this.robots = new RobotResourceManager(resources.robots || {});
    this.logos = new LogoResourceManager(resources.logos || {});
    if (resources.dispensers) {
      this.dispensers = new DispenserResourceManager(resources.dispensers);
    }
    /**
     * - helpLink by default leads to https://osrf.github.io/ros2multirobotbook/rmf-core.html
          It is configurable in the dashboard resources.
       - reportIssue default leads to https://github.com/open-rmf/rmf-web/issues
          It is configurable in dashboard resources.
       - Both variables are hardcoded for now because the url is public.
     */
    this.helpLink = resources.helpLink || 'https://osrf.github.io/ros2multirobotbook/rmf-core.html';
    this.reportIssue = resources.reportIssue || 'https://github.com/open-rmf/rmf-web/issues';
  }
}

export const logoSize = 180;
export const appDrawerWidth = 240;
