import { RmfLauncher } from '../rmf-launcher';

describe('smoke test', () => {
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  it('can load', () => {
    browser.url('/');
    expect($('#schedule-visualizer')).toBeVisible();
  });
});
