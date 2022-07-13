import { render } from '@testing-library/react';
import React from 'react';
import { makeLayout } from './test-utils.spec';
import { Window } from './window';
import { WindowContainer } from './window-container';

describe('Window', () => {
  it('smoke test', () => {
    const layout = makeLayout();
    expect(() =>
      render(
        <WindowContainer layout={layout}>
          {layout.map(({ i }) => (
            <Window key={i} title={`Window ${i}`} />
          ))}
        </WindowContainer>,
      ),
    ).not.toThrow();
  });
});
