import { act, render as render_, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { ColorContext, ColorManager, RobotMarker } from '../../lib';
import { makeRobot } from './test-utils';

describe('robot-markers', () => {
  let colorManager: ColorManager;
  let fakeOnClick: ReturnType<typeof jasmine.createSpy>;

  beforeEach(() => {
    colorManager = new ColorManager();
    fakeOnClick = jasmine.createSpy();
    // FIXME: karma should support the apis needed. So we wouldn't need to define the robotPrimaryColor
    colorManager.robotPrimaryColor = jasmine.createSpy();
  });

  async function render(Component: JSX.Element): Promise<RenderResult> {
    const renderImpl = () => {
      return render_(
        <ColorContext.Provider value={colorManager}>
          <svg>{Component}</svg>
        </ColorContext.Provider>,
      );
    };

    let root: ReturnType<typeof renderImpl>;
    await act(async () => {
      root = render_(
        <ColorContext.Provider value={colorManager}>
          <svg>{Component}</svg>
        </ColorContext.Provider>,
      );
    });

    return root!;
  }

  it('trigger onClick event', async () => {
    const root = await render(
      <RobotMarker
        robot={makeRobot()}
        fleetName="test_fleet"
        footprint={1}
        onClick={fakeOnClick}
        data-testid="marker"
      />,
    );
    userEvent.click(root.getByTestId('marker'));
    expect(fakeOnClick).toHaveBeenCalled();
  });

  it('providing iconPath renders an image', async () => {
    const root = await render(
      <RobotMarker robot={makeRobot()} fleetName="test_fleet" footprint={1} iconPath="test_icon" />,
    );
    expect(root.container.querySelector('image')).not.toBeNull();
  });

  it("uses ColorManager to determine robot's color", async () => {
    await render(<RobotMarker robot={makeRobot()} fleetName="test_fleet" footprint={1} />);
    expect(colorManager.robotPrimaryColor).toHaveBeenCalled();
  });
});
