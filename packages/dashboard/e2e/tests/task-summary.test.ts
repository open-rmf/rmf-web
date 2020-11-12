import { makeLauncher } from '../rmf-launcher';
import { login, openRequestForm, overwriteClick, requestLoop } from './utils';

describe('Loop request for task summary', () => {
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  it('renders task summary', () => {
    browser.setTimeout({ script: 120000 });
    openRequestForm();
    requestLoop({
      pointA: 'pantry',
      pointB: 'cubicle_1',
    });
    const backButton = $('[name="back-button"]');
    backButton.click();

    $('[data-component=MainMenu] [data-item=Plans]').click();

    browser.waitUntil(() => $('[data-component=TreeItem]').isDisplayed() === true, {
      timeout: 60000,
      timeoutMsg: 'expected TreeItem to be not null!',
    });

    const treeItem = $('[data-component=TreeItem]');
    expect(treeItem).toBeVisible();
    treeItem.click();
  });
});
