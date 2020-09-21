import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import DashboardTour from '../tour';
import { SpotlightValue } from '../spotlight-value';
import { ReactWrapper } from 'enzyme';
import { OmniPanelViewIndex } from '../dashboard';

/*react-leaflet does not work well on jsdom,
suppress warnings of not being able to find elements from leaflet*/
jest.spyOn(console, 'warn').mockImplementation(() => {});

const mount = createMount();

describe('Dashboard Tour', () => {
  let tourState: boolean = true;
  let setTourState: jest.Mock = jest.fn();
  let setTourShowOmniPanel: jest.Mock = jest.fn();
  let setTourSettingsAndOmniPanel: jest.Mock = jest.fn();
  let doorSpotlight: SpotlightValue<string> = { value: 'main_door' };
  let setDoorSpotlight: jest.Mock = jest.fn();

  let root: ReactWrapper;
  let expectedText: string;
  let stepText: string;

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
  });

  afterEach(() => {
    root?.unmount();
  });

  const wait = (delay = 0) => new Promise((resolve) => setTimeout(resolve, delay));

  it('renders without crashing', () => {
    expect(root.html()).toMatchSnapshot();
  });

  it('renders first welcome step (1/12)', () => {
    expectedText = 'Welcome to RoMi dashboard';
    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders zoom button step (2/12)', async () => {
    expectedText =
      'Click on the zoom buttons to change the view of the floor plan.Alternatively, the scroll button on your mouse would work too!';
    root.find('#tour-next-btn').first().simulate('click');
    await wait(5);

    setTimeout(() => (stepText = root.find('.reactour__helper').hostNodes().text()), 100);

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders floor plan button step (3/12)', async () => {
    expectedText =
      'Use the floor plan button to switch between available levels and enabling / disabling the view of different components.';
    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 3; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    setTimeout(() => (stepText = root.find('.reactour__helper').hostNodes().text()), 100);

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders leaflet step (4/12)', async () => {
    expectedText =
      'Clicking individual components like doors, robots, lifts on the map will open up its corresponding information tab in the omnipanel.';
    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 4; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    setTimeout(() => (stepText = root.find('.reactour__helper').hostNodes().text()), 100);

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders omnipanel button step (5/12)', async () => {
    expectedText =
      'The Omnipanel Button shows the different panel options available in the dashboard. Clicking each item would list different information about it!';

    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 5; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders main menu step (6/12)', async () => {
    expectedText =
      'Each Panel contains a list of the available items and their corresponding states.';

    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 6; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    setTimeout(() => (stepText = root.find('.reactour__helper').hostNodes().text()), 100);

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders door panel step (7/12)', async () => {
    expectedText = `Let us take a look into the 🚪 Doors Panel.`;

    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 7; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders door tab step (8/12)', async () => {
    expectedText = 'Here is an example of what you will see when a door tab is expanded';

    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 8; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders commands panel step (9/12)', async () => {
    expectedText =
      'The Commands Panel allows you to send different types of requests that will be handled by RoMi.';

    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 9; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders loop request step (10/12)', async () => {
    expectedText =
      'An example is the Loop Request which can be iterated multiple times. RoMi will assign the most suitable robot to perform the task at the point of request.';

    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 9; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders settings button step (11/12)', async () => {
    expectedText = 'The Settings Button opens up the drawer for different dashboard settings.';

    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 11; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });

  it('renders traj anims step (12/12)', async () => {
    expectedText =
      'Finally, Trajectory Animations can be changed using the options available. Look out for new features ahead!';

    const nextBtn = root.find('#tour-next-btn').first();
    for (let i = 1; i < 12; i++) {
      nextBtn.simulate('click');
      await wait(5);
    }

    stepText = root.find('.reactour__helper').hostNodes().text();

    expect(stepText.includes(expectedText));
    expect(root.html()).toMatchSnapshot();
  });
});
