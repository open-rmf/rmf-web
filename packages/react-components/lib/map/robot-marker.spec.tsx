import { act, cleanup, render as render_, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { makeRobot } from '../robots/test-utils.spec';
import { RobotMarker } from './robot-marker';
import { RobotData } from './robots-overlay';
import { makeRobotData } from './test-utils.spec';

describe('robot-markers', () => {
  async function render(Component: JSX.Element): Promise<RenderResult> {
    let root: RenderResult;
    await act(async () => {
      root = render_(<svg>{Component}</svg>);
    });

    return root!;
  }

  it('smoke test', async () => {
    const robots: RobotData[] = [
      makeRobotData({
        name: 'test_robot_1',
        state: makeRobot({ name: 'test_robot_1' }),
        inConflict: false,
      }),
      makeRobotData({
        name: 'test_robot_2',
        state: makeRobot({ name: 'test_robot_2' }),
        inConflict: true,
      }),
    ];
    for (const robot of robots) {
      await render(
        <RobotMarker
          fleet={robot.fleet}
          name={robot.name}
          model={robot.model}
          state={robot.state}
          footprint={robot.footprint}
          color="#000000"
        />,
      );
      cleanup();
    }
  });

  it('trigger onClick event', async () => {
    const robot = makeRobotData();
    const onClick = jasmine.createSpy();
    const root = await render(
      <RobotMarker
        fleet={robot.fleet}
        name={robot.name}
        model={robot.model}
        state={robot.state}
        footprint={robot.footprint}
        color="#000000"
        onClick={onClick}
      />,
    );
    userEvent.click(root.getByTestId('marker'));
    expect(onClick).toHaveBeenCalled();
  });

  it('providing iconPath renders an image', async () => {
    const robot = makeRobotData();
    const root = await render(
      <RobotMarker
        fleet={robot.fleet}
        name={robot.name}
        model={robot.model}
        state={robot.state}
        footprint={robot.footprint}
        color="#000000"
        iconPath="test_icon"
      />,
    );
    expect(root.container.querySelector('image')).not.toBeNull();
  });
});
