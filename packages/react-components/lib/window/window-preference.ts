import { Layout } from 'react-grid-layout';
import { v4 as uuidv4 } from 'uuid';

export declare namespace WindowPreference {
  type BaseLayout = Pick<Layout, 'w' | 'h' | 'minW' | 'minH' | 'maxW' | 'maxH'>;
}

export class WindowPreference {
  constructor(public baseLayout: WindowPreference.BaseLayout) {}

  /**
   * Creates a `Layout` that can be used in `WindowManager`.
   */
  createLayout(keyPrefix?: string): Layout {
    return {
      i: `${keyPrefix ? keyPrefix + '-' : ''}${uuidv4()}`,
      x: 0,
      y: 0,
      ...this.baseLayout,
    };
  }
}
