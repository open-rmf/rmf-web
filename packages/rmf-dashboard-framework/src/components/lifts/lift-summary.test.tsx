import { waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import type { Lift, LiftState } from 'api-client';
import React, { act } from 'react';
import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { LiftSummary } from './lift-summary';
import { makeLift, makeLiftState } from './test-utils.test';

describe('LiftSummary', () => {
  const mockLift: Lift = makeLift({ name: 'test_lift' });

  const rmfApi = new MockRmfApi();
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders lift summary correctly', async () => {
    const onCloseMock = vi.fn();
    const root = render(
      <Base>
        <LiftSummary onClose={onCloseMock} lift={mockLift} />
      </Base>,
    );

    // Create the subject for the lift
    rmfApi.getLiftStateObs('test_lift');
    const mockLiftState: LiftState = makeLiftState({
      lift_name: 'test_lift',
      destination_floor: 'L2',
    });
    act(() => {
      rmfApi.liftStateObsStore['test_lift'].next(mockLiftState);
    });

    expect(root.getByText('Name')).toBeTruthy();
    expect(root.getByText('Current Floor')).toBeTruthy();
    expect(root.getByText('Destination Floor')).toBeTruthy();
    expect(root.getByText('State')).toBeTruthy();
    expect(root.getByText('Session ID')).toBeTruthy();

    expect(root.getByText('test_lift')).toBeTruthy();
    expect(root.getByText('L1')).toBeTruthy();
    expect(root.getByText('L2')).toBeTruthy();
    expect(root.getByText('CLOSED')).toBeTruthy();
    expect(root.getByText('test_session')).toBeTruthy();

    userEvent.keyboard('{Escape}');
    await waitFor(() => expect(onCloseMock).toHaveBeenCalledTimes(1));
  });
});
