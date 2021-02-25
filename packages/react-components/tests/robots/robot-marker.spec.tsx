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
  });

  async function render(Component: JSX.Element): Promise<RenderResult> {
    let root: RenderResult;
    await act(async () => {
      root = render_(
        <ColorContext.Provider value={colorManager}>
          <svg>{Component}</svg>
        </ColorContext.Provider>,
      );
      // default marker sets the color using an async function, need this to ensure
      // the promise is resolved.
      await new Promise((res) => setTimeout(res, 0));
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
});
