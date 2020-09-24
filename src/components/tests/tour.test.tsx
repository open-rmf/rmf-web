import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import DashboardTour from '../tour/tour';
import { SpotlightValue } from '../spotlight-value';
import { ReactWrapper } from 'enzyme';
import { OmniPanelViewIndex } from '../dashboard';
import { tourText } from '../tour/tour-text';

/*react-leaflet does not work well on jsdom,
suppress warnings of not being able to find elements from leaflet*/
jest.spyOn(console, 'warn').mockImplementation(() => {});

const mount = createMount();

describe('Dashboard Tour', () => {
  const tourState: boolean = true;
  const setTourState: jest.Mock = jest.fn();
  const setTourShowOmniPanel: jest.Mock = jest.fn();
  const setTourSettingsAndOmniPanel: jest.Mock = jest.fn();
  const doorSpotlight: SpotlightValue<string> = { value: 'main_door' };
  const setDoorSpotlight: jest.Mock = jest.fn();

  let root: ReactWrapper;
  let expectedText: string;
  let stepText: string;
  let nextBtn: ReactWrapper;

  beforeEach(() => {
    const tourProps = {
      tourState,
      setTourState,
      setTourShowOmniPanel,
      setTourSettingsAndOmniPanel,
      OmniPanelViewIndex,
      doorSpotlight,
      setDoorSpotlight,
    };
    root = mount(<DashboardTour tourProps={tourProps} />);
    nextBtn = root.find('#tour-next-btn').first();
  });

  afterEach(() => {
    root?.unmount();
  });

  const wait = (delay = 0) => new Promise((resolve) => setTimeout(resolve, delay));

  it('renders without crashing', () => {
    expect(root.html()).toMatchSnapshot();
  });

  it('renders first welcome step (1/12)', () => {
    expectedText = tourText.step1Welcome;
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders zoom button step (2/12)', async () => {
    expectedText = tourText.step2Zoom;
    nextBtn.simulate('click');
    await wait(5);

    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders floor plan button step (3/12)', async () => {
    expectedText = tourText.step3Floorplan;

    for (let i = 1; i < 3; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders leaflet step (4/12)', async () => {
    expectedText = tourText.step4Leaflet;

    for (let i = 1; i < 4; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders omnipanel button step (5/12)', async () => {
    expectedText = tourText.step5Omnipanel;

    for (let i = 1; i < 5; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders main menu step (6/12)', async () => {
    expectedText = tourText.step6MainMenu;

    for (let i = 1; i < 6; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders door panel step (7/12)', async () => {
    expectedText = tourText.step7DoorPanel;

    for (let i = 1; i < 7; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders door tab step (8/12)', async () => {
    expectedText = tourText.step8DoorTab;

    for (let i = 1; i < 8; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders commands panel step (9/12)', async () => {
    expectedText = tourText.step9CommandsPanel;

    for (let i = 1; i < 9; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders loop request step (10/12)', async () => {
    expectedText = tourText.step10LoopRequest;

    for (let i = 1; i < 9; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders settings button step (11/12)', async () => {
    expectedText = tourText.step11Settings;

    for (let i = 1; i < 11; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders traj anims step (12/12)', async () => {
    expectedText = tourText.step12TrajAnims;

    for (let i = 1; i < 12; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });
});
