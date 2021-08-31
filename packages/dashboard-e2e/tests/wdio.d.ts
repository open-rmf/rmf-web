declare namespace WebdriverIO {
  interface OverwriteClickOptions extends ClickOptions {
    force?: boolean;
  }

  interface Element {
    click(options?: OverwriteClickOptions): void;
  }
}
