import { RmfLauncher } from '../rmf-launcher';
import { overwriteClick } from './utils';

describe('ui interactions', () => {
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  it('clicking a door on the map focuses it on the panel', () => {
    const door = $(`[data-component=Door]`);
    const doorName = door.getAttribute('aria-label');
    door.waitForClickable();
    door.click();

    expect($(`[data-component=DoorItem][data-name=${doorName}] [data-role=details]`)).toBeVisible();
  });

  it('clicking a robot on the map focuses it on the panel', () => {
    const robot = $('[data-component=Robot]');
    const robotName = robot.getAttribute('aria-label');
    robot.click();

    expect($(`[data-component=RobotItem][data-name=${robotName}] [data-role=details]`)).toBeVisible();
  });
});
