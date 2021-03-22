import { makeLauncher } from '../../rmf-launcher';
import { login, overwriteClick } from './utils';

describe('Simple Filter', () => {
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  before(() => {
    $('[data-component=MainMenu] [data-item=Doors]').click();
  });

  it('Clicking on an equipment on the map resets the filter', () => {
    // set value to filter input
    $('[data-component=simple-filter]').$('input').setValue('value');

    // get door marker and click
    const door = $('[data-component=DoorMarker]');
    const doorName = door.getAttribute('aria-label');

    door.waitForClickable();
    door.click();

    // check that the filter is empty after clicking
    expect($('[data-component=simple-filter]').$('input').getValue()).toEqual('');
    // check that door panel is expanded
    expect($(`.MuiAccordion-root*=${doorName}`).$('.MuiAccordionDetails-root')).toBeVisible();
  });
});
