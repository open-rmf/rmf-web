import { ColorManager } from 'react-components';

export class NegotiationColors extends ColorManager {
  /**
   * Always return fixed value.
   */
  async robotPrimaryColor(...whatever: unknown[]): Promise<string> {
    return 'orange';
  }

  robotColorFromCache(...whatever: unknown[]): string {
    return 'orange';
  }
}
