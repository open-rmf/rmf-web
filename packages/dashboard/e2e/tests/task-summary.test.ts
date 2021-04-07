import { makeLauncher } from '../../rmf-launcher';
import { login, overwriteClick } from './utils';
import { execSync } from 'child_process';

describe('Loop request for task summary', () => {
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  it('renders task summary', () => {
    browser.setTimeout({ script: 120000 });
    execSync('ros2 run rmf_demos_tasks dispatch_loop -s coe -f lounge -n 1 --use_sim_time');

    $('[data-component=MainMenu] [data-item=Plans]').click();

    browser.waitUntil(() => $('[role="treeitem"]').isDisplayed() === true, {
      timeout: 120000,
      timeoutMsg: 'expected TreeItem to be not null!',
    });

    const treeItem = $('[role="treeitem"]');
    expect(treeItem).toBeVisible();
    treeItem.click();
  }).timeout(180000);
});
