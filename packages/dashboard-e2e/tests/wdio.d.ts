import { ClickOptions } from 'webdriverio/build/types';

declare namespace WebdriverIO {
  interface OverwriteClickOptions extends ClickOptions {
    force?: boolean;
  }

  interface Element {
    click(options?: OverwriteClickOptions): Promise<void>;
  }
}
