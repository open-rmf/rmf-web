import { RmfLauncher } from '../rmf-launcher';
import { overwriteClick } from './utils';

describe('loop request', () => {
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  it('rmf responds to loop request', () => {
    $('[data-component=MainMenu] [data-item=Robots]').click();
    const robotItem = $('[data-component=RobotItem]');
    robotItem.click();
    const position = robotItem.$('[data-role=position]').getText();
    robotItem.$('button=Loop').click();
    robotItem.$('input[name=numLoops]').setValue(1);
    robotItem.$('button=Request').click();
    robotItem.$('button=Info').click();
    expect(robotItem.$('[data-role=position]')).not.toHaveText(position);
  });
});
