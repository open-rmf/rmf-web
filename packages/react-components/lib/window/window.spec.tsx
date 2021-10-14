import { render } from '@testing-library/react';
import React from 'react';
import { makeLayouts } from './test-utils.spec';
import { Window } from './window';
import { WindowContainer } from './window-container';

describe('Window', () => {
  it('smoke test', () => {
    const layouts = makeLayouts();
    expect(() =>
      render(
        <WindowContainer layouts={layouts}>
          {layouts.map(({ i }) => (
            <Window key={i} title={`Window ${i}`} />
          ))}
        </WindowContainer>,
      ),
    ).not.toThrow();
  });
});
