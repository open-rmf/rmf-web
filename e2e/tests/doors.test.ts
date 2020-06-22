import { RmfLauncher } from '../rmf-launcher';

describe('door tests', () => {
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => browser.url('/'));

  it('clicking door on the map focuses it on the panel', () => {
    const doorName = 'main_door';

    browser.$(`#svg-g-door-${doorName}`).click();
    const doorSummary = browser
      .react$$('ExpansionPanel')
      .find(elem => elem.react$$('Typography').find(elem => elem.getText() === doorName))!;
    const doorDetail = doorSummary.react$('ExpansionPanelDetails');
    expect(doorDetail).toBeVisible();
  });
});
