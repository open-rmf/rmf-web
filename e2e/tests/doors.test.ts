import { RmfLauncher } from '../rmf-launcher';
import { overwriteClick } from './utils';

describe('door tests', () => {
  const doorName = 'main_door';
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  it('clicking door on the map focuses it on the panel', () => {
    $(`#svg-g-door-${doorName}`).click();

    const doorItem = $(`#DoorItem*=${doorName}`);
    expect(doorItem.$('p*=Type')).toBeVisible();
  });

  it('clicking on open button opens the door', () => {
    const doorItem = $(`#DoorItem*=${doorName}`);

    expect(doorItem.$('span=CLOSED')).toBeVisible();

    doorItem.$('button=Open').click();

    expect(doorItem.$('span=OPEN')).toBeVisible();
  });

  it('clicking on close button closes the door', () => {
    const doorItem = $(`#DoorItem*=${doorName}`);

    expect(doorItem.$('span=OPEN')).toBeVisible();

    doorItem.$('button=Close').click();

    expect(doorItem.$('span=CLOSED')).toBeVisible();
  });
});
