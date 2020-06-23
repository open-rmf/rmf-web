import { RmfLauncher } from '../rmf-launcher';
import { overwriteClick } from './utils';

/**
 * FIXME: Some tests are skipped in headless mode as they try to click a svg element. svg elements
 * are not rendered in headless mode and it is not possible to click them.
 */
describe('ui interactions', () => {
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  it('clicking a door on the map focuses it on the panel', () => {
    if (process.env.CI) {
      console.warn('"clicking a door on the map focuses it on the panel" test skipped because svg is not rendered in headless mode')
      return;
    }
    const door = $(`[data-component=Door]`);
    const doorName = door.getAttribute('aria-label');
    door.waitForClickable();
    door.click();

    expect($(`[data-component=DoorItem][data-name=${doorName}] [data-role=details]`)).toBeVisible();
  });

  it('clicking a robot on the map focuses it on the panel', () => {
    if (process.env.CI) {
      console.warn('"clicking a robot on the map focuses it on the panel" test skipped because svg is not rendered in headless mode')
      return;
    }
    const robot = $('[data-component=Robot]');
    const robotName = robot.getAttribute('aria-label');
    robot.click();

    expect($(`[data-component=RobotItem][data-name=${robotName}] [data-role=details]`)).toBeVisible();
  });
});
