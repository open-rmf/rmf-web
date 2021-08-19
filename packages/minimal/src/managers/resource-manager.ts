import axios from 'axios';
import { LogoResourceManager, LogoResource } from './resource-manager-logos';

export const RESOURCE_PREFIX = process.env.PUBLIC_URL || '';

export interface ResourceConfigurationsType {
  logos?: Record<string, LogoResource>;
}

export default class ResourceManager {
  logos: LogoResourceManager | undefined;

  static getResourceConfigurationFile = async (): Promise<ResourceManager | undefined> => {
    try {
      // Gets data served by the project itself
      const response = await axios.get(RESOURCE_PREFIX + '/assets/icons/main.json');
      const resources = response.data as ResourceConfigurationsType;
      return ResourceManager.resourceManagerFactory(resources);
    } catch (error) {
      console.error(error);
      return undefined;
    }
  };

  static resourceManagerFactory = (
    resources: ResourceConfigurationsType | undefined,
  ): ResourceManager => {
    if (!resources) {
      return {} as ResourceManager;
    }

    if (resources?.logos && !Object.keys(resources.logos).length) {
      const data = Object.assign({}, resources);
      delete data['logos'];
      return new ResourceManager(data as ResourceConfigurationsType);
    }

    return new ResourceManager(resources as ResourceConfigurationsType);
  };

  constructor(resources: ResourceConfigurationsType) {
    if (resources.logos) {
      this.logos = new LogoResourceManager(resources.logos);
    }
  }
}
