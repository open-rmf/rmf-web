import { render } from '@testing-library/react';
import React from 'react';
import { makeLayout } from './test-utils.spec';
import { Window } from './window';
import { WindowManager } from './window-manager';

describe('Window', () => {
  it('smoke test', () => {
    const layout = makeLayout();
    expect(() =>
      render(
        <WindowManager layout={layout}>
          {layout.map(({ i }) => (
            <Window key={i} title={`Window ${i}`} />
          ))}
        </WindowManager>,
      ),
    ).not.toThrow();
  });
});
