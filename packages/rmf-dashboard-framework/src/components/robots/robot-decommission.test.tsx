import { fireEvent } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { RobotDecommissionButton } from './robot-decommission';
import { makeRobot } from './test-utils.test';

describe('Robot decommission button', () => {
  const rmfApi = new MockRmfApi();
  // mock out some api calls so they never resolves
  const mockDecommission = vi.fn().mockImplementation(async () => {
    // Simulate some asynchronous operation that has no return value
    await new Promise((resolve) => setTimeout(resolve, 100));
  });
  const mockRecommission = vi.fn().mockImplementation(async () => {
    // Simulate some asynchronous operation that has no return value
    await new Promise((resolve) => setTimeout(resolve, 100));
  });
  rmfApi.fleetsApi.decommissionRobotFleetsNameDecommissionPost = mockDecommission;
  rmfApi.fleetsApi.recommissionRobotFleetsNameRecommissionPost = mockRecommission;
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('Renders decommission button', () => {
    const root = render(
      <Base>
        <RobotDecommissionButton
          fleet="test_fleet"
          robotState={makeRobot({ name: 'test_robot' })}
        />
      </Base>,
    );

    expect(root.getByText('Decommission')).toBeTruthy();
    fireEvent.click(root.getByText('Decommission'));
    expect(root.getByText('Decommission [test_fleet:test_robot]')).toBeTruthy();
    expect(root.getByText('Confirm'));
    fireEvent.click(root.getByText('Confirm'));
    expect(mockDecommission).toHaveBeenCalledOnce();
  });

  it('Renders recommission button', () => {
    const root = render(
      <Base>
        <RobotDecommissionButton
          fleet="test_fleet"
          robotState={makeRobot({
            name: 'test_robot',
            commission: {
              dispatch_tasks: false,
              direct_tasks: false,
              idle_behavior: false,
            },
          })}
        />
      </Base>,
    );

    expect(root.getByText('Recommission')).toBeTruthy();
    fireEvent.click(root.getByText('Recommission'));
    expect(root.getByText('Recommission [test_fleet:test_robot]')).toBeTruthy();
    expect(root.getByText('Confirm'));
    fireEvent.click(root.getByText('Confirm'));
    expect(mockRecommission).toHaveBeenCalledOnce();
  });
});
