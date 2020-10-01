import { createMount, createShallow } from '@material-ui/core/test-utils';
import React from 'react';
import DashboardTour from '../tour/tour';
import { SpotlightValue } from '../spotlight-value';
import { ReactWrapper, ShallowWrapper } from 'enzyme';
import { OmniPanelViewIndex } from '../dashboard';
import { tourText } from '../tour/tour-data';

/*react-leaflet does not work well on jsdom,
suppress warnings of not being able to find elements from leaflet*/
jest.spyOn(console, 'warn').mockImplementation(() => {});

const mount = createMount();
const shallow = createShallow();

describe('Dashboard Tour', () => {
  const tourState: boolean = true;
  const setTourState: jest.Mock = jest.fn();
  const setShowSettings: jest.Mock = jest.fn();
  const setShowOmniPanel: jest.Mock = jest.fn();
  const setShowHelp: jest.Mock = jest.fn();
  const clearSpotlights: jest.Mock = jest.fn();
  const setCurrentView: jest.Mock = jest.fn();
  const doorSpotlight: SpotlightValue<string> = { value: 'main_door' };
  const setDoorSpotlight: jest.Mock = jest.fn();

  let root: ReactWrapper;
  let expectedText: string;
  let stepText: string;
  let nextBtn: ReactWrapper;
  const tourProps = {
    tourState,
    setTourState,
    setShowSettings,
    setShowOmniPanel,
    setShowHelp,
    clearSpotlights,
    setCurrentView,
    doorSpotlight,
    setDoorSpotlight,
  };
  const wait = (delay = 0) => new Promise((resolve) => setTimeout(resolve, delay));

  beforeEach(() => {
    root = mount(<DashboardTour tourProps={tourProps} />);
    nextBtn = root.find('[data-testid="next-btn"]').first();
  });

  afterEach(() => {
    root?.unmount();
  });

  it('renders without crashing', () => {
    const tour = shallow(<DashboardTour tourProps={tourProps} />);
    expect(tour).toMatchSnapshot();
  });

  it('renders first welcome step', () => {
    expectedText = tourText.step1Welcome;
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders zoom button step', async () => {
    expectedText = tourText.step2Zoom;
    nextBtn.simulate('click');
    await wait(5);

    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders floor plan button step', async () => {
    expectedText = tourText.step3Floorplan;

    for (let i = 1; i < 3; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders leaflet step', async () => {
    expectedText = tourText.step4Leaflet;

    for (let i = 1; i < 4; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders omnipanel button step', async () => {
    expectedText = tourText.step5Omnipanel;

    for (let i = 1; i < 5; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders main menu step', async () => {
    expectedText = tourText.step6MainMenu;

    for (let i = 1; i < 6; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders door panel step', async () => {
    expectedText = tourText.step7DoorPanel;

    for (let i = 1; i < 7; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders door tab step', async () => {
    expectedText = tourText.step8DoorTab;

    for (let i = 1; i < 8; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders commands panel step', async () => {
    expectedText = tourText.step9CommandsPanel;

    for (let i = 1; i < 9; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders loop request step', async () => {
    expectedText = tourText.step10LoopRequest;

    for (let i = 1; i < 10; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders settings button step', async () => {
    expectedText = tourText.step11Settings;

    for (let i = 1; i < 11; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders traj anims step', async () => {
    expectedText = tourText.step12TrajAnims;

    for (let i = 1; i < 12; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders help button step', async () => {
    expectedText = tourText.step13HelpButton;

    for (let i = 1; i < 13; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('renders help drawer step', async () => {
    expectedText = tourText.step14HelpDrawer;

    for (let i = 1; i < 14; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('[data-testid="stepBox"]').first().text();

    expect(stepText.includes(expectedText));
  });

  it('closes the tour when last step button is clicked', async () => {
    for (let i = 1; i < 15; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    root.find('[data-testid="last-step-btn"]').first().simulate('click');

    expect(setTourState).toHaveBeenCalledTimes(1);
  });
});
