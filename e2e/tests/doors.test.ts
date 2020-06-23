import { RmfLauncher } from '../rmf-launcher';

describe('door tests', () => {
  const doorName = 'main_door';
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => browser.url('/'));

  it('clicking door on the map focuses it on the panel', () => {
    $(`#svg-g-door-${doorName}`).click();

    const doorItem = $(`#DoorItem*=${doorName}`);
    expect(doorItem.$('p*=Type')).toBeVisible();
  });

  // TODO: fixed with feat/addLiftRequestForm, uncomment after merging
  // it('clicking on open button opens the door', () => {
  //   const doorItem = $(`#DoorItem*=${doorName}`);

  //   expect(doorItem.$('span=CLOSED')).toBeVisible();

  //   const openButton = doorItem.$('button=Open');
  //   openButton.click();

  //   expect(doorItem.$('span=OPEN')).toBeVisible();
  // });
});
