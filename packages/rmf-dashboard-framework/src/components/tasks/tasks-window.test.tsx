import { fireEvent, screen } from '@testing-library/react';
import React, { act } from 'react';
import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { AppEvents } from '../app-events';
import { TasksWindow } from './tasks-window';

vi.mock('../app-events', () => ({
  AppEvents: {
    refreshTaskApp: {
      subscribe: vi.fn(() => ({ unsubscribe: vi.fn() })),
      next: vi.fn(),
    },
  },
}));

describe('Tasks window', () => {
  const rmfApi = new MockRmfApi();
  rmfApi.tasksApi.queryTaskStatesTasksGet = vi.fn().mockResolvedValue({ data: [] });

  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders without crashing', () => {
    const root = render(
      <Base>
        <TasksWindow />
      </Base>,
    );
    expect(root.getByText('Tasks')).toBeTruthy();
  });

  it('triggers task refresh when Refresh button is clicked', () => {
    render(<TasksWindow onClose={() => {}} />);
    const refreshButton = screen.getByTestId('refresh-button');
    act(() => {
      fireEvent.click(refreshButton);
    });
    expect(AppEvents.refreshTaskApp.next).toHaveBeenCalled();
  });
});
