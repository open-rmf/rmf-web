import { screen } from '@testing-library/react';
import React from 'react';
import { beforeEach, describe, expect, it } from 'vitest';

import { RmfApiProvider } from '../hooks';
import { MockRmfApi, render, TestProviders } from '../utils/test-utils.test';
import { MicroAppManifest } from './micro-app';
import { InitialWindow, LocallyPersistentWorkspace, Workspace } from './workspace';

const mockMicroApp: MicroAppManifest = {
  Component: () => <div>Mock App</div>,
  appId: 'mock-app',
  displayName: 'Mock App',
};

describe('workspace', () => {
  const rmfApi = new MockRmfApi();

  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders without crashing', () => {
    render(
      <Base>
        <Workspace initialWindows={[]} allowDesignMode={false} appRegistry={[]} />
      </Base>,
    );
  });

  it('renders initial windows', () => {
    const initialWindows: InitialWindow[] = [
      {
        layout: { x: 0, y: 0, w: 1, h: 1 },
        microApp: mockMicroApp,
      },
    ];
    render(<Workspace initialWindows={initialWindows} />);
    expect(screen.getByText('Mock App')).toBeTruthy();
  });
});

describe('LocallyPersistentWorkspace', () => {
  const storageKey = 'test-workspace';

  beforeEach(() => {
    localStorage.clear();
  });

  it('loads initial windows from localStorage', () => {
    const savedLayout = [
      {
        layout: { i: 'window-0', x: 0, y: 0, w: 1, h: 1 },
        appId: 'mock-app',
      },
    ];
    localStorage.setItem(storageKey, JSON.stringify(savedLayout));

    render(
      <LocallyPersistentWorkspace
        defaultWindows={[]}
        storageKey={storageKey}
        appRegistry={[mockMicroApp]}
      />,
    );

    expect(screen.getByText('Mock App')).toBeTruthy();
  });

  it('falls back to default windows when localStorage is empty', () => {
    const defaultWindows: InitialWindow[] = [
      {
        layout: { x: 0, y: 0, w: 1, h: 1 },
        microApp: mockMicroApp,
      },
    ];
    render(
      <LocallyPersistentWorkspace
        defaultWindows={defaultWindows}
        storageKey={storageKey}
        appRegistry={[mockMicroApp]}
      />,
    );

    expect(screen.getByText('Mock App')).toBeTruthy();
  });
});
