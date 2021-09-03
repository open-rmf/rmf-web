import { execSync } from 'child_process';

describe('Loop request for negotiations', () => {
  before(() => {
    browser.setTimeout({ script: 120000 });
  });

  it('renders negotiation trajectory', async () => {
    execSync('ros2 launch rmf_demos office_conflict.launch.xml');

    await (await $('[data-component=MainMenu] [data-item=Negotiations]')).click();

    await browser.waitUntil(
      async () => (await (await $('[data-component=TreeItem]')).isDisplayed()) === true,
      {
        timeout: 60000,
        timeoutMsg: 'expected TreeItem to be not null!',
      },
    );

    const treeItem = await $('[data-component=TreeItem]');
    expect(treeItem).toBeDisplayed();
    treeItem.click();

    const subTreeItem = await treeItem.$('[data-component=TreeItem]');
    expect(subTreeItem).toBeDisplayed();
    subTreeItem.click();
  }).timeout(300000);

  // Disabled till we find out a way to locate that floor icon
  // Also figure out a way for a non-rejected negotiation status click

  // it('renders negotiation trajectory', () => {
  //   //$('[data-component=viewOptions]').moveTo();
  //   $('[data-component=NegotiationTrajCheckbox]').click();
  //   expect($('[data-component=RobotTrajectory]')).toBeVisible();
  // });
});
