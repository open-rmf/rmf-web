import { render } from '@testing-library/react';
import React from 'react';
import { makeLayouts } from './test-utils.spec';
import { Window } from './window';
import { WindowManager } from './window-manager';

describe('Window', () => {
  it('smoke test', () => {
    const layouts = makeLayouts();
    expect(() =>
      render(
        <WindowManager layouts={layouts}>
          {layouts.map(({ i }) => (
            <Window key={i} title={`Window ${i}`} />
          ))}
        </WindowManager>,
      ),
    ).not.toThrow();
  });
});
