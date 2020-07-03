import { RmfLauncher } from '../rmf-launcher';
import { overwriteClick, removeTextFromAutocomplete } from './utils';

describe('Delivery request', () => {
    const launcher = new RmfLauncher();

    before(async () => await launcher.launch());
    after(async () => await launcher.kill());

    before(() => overwriteClick());
    before(() => browser.url('/'));

    it('rmf responds to delivery request', () => {
        $('[data-component=MainMenu] [data-item=Robots]').click();
        const robotItem = $('[data-component=RobotItem]');
        robotItem.click();
        const position = robotItem.$('[data-role=position]').getText();
        robotItem.$('button=Delivery').click();
        // .clearValue it's not working on Autocomplete
        robotItem.$('input[name=pickupPlace]').setValue(removeTextFromAutocomplete(10));
        robotItem.$('input[name=pickupPlace]').setValue('pantry');
        $('.MuiAutocomplete-popper').click()

        robotItem.$('input[name=pickupDispenser]').click();
        $('.MuiAutocomplete-popper').click()

        robotItem.$('input[name=dropoffPlace]').setValue(removeTextFromAutocomplete(20));
        robotItem.$('input[name=dropoffPlace]').setValue('hardware_2');
        $('.MuiAutocomplete-popper').click()

        robotItem.$('input[name=dropoffDispenser]').click();
        $('.MuiAutocomplete-popper').click()

        robotItem.$('button=Request').click();
        robotItem.$('button=Info').click();
        expect(robotItem.$('[data-role=position]')).not.toHaveText(position);
    });

    it('renders robot trajectory', () => {
        expect($('[data-component=RobotTrajectory]')).toBeVisible();
    });
});

